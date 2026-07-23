import rclpy
from rclpy.node import Node
from nav_msgs.msg import Odometry
from actuator_msgs.msg import Actuators
from geometry_msgs.msg import PoseStamped, Point as GeometryPoint, TwistStamped
from visualization_msgs.msg import Marker, MarkerArray
import math
import csv
import os
import numpy as np
import yaml

from .solver_pid_lqr import PIDLQRSolver

class PID:
    def __init__(self, Kp, Ki, Kd, dt, out_min=-np.inf, out_max=np.inf):
        self.Kp = Kp
        self.Ki = Ki
        self.Kd = Kd
        self.dt = dt
        self.out_min = out_min
        self.out_max = out_max
        self.integral = 0
        self.prev_error = 0
        
    def compute(self, error, dt=None, reset_derivative=False):
        if dt is not None:
            self.dt = dt
            
        proportional = self.Kp * error
        if reset_derivative:
            self.prev_error = error
        derivative = self.Kd * (error - self.prev_error) / self.dt
        self.prev_error = error
        
        self.integral += error * self.dt
        output = proportional + self.Ki * self.integral + derivative
        
        # Anti-windup (Dynamic Back-Calculation / Clamping)
        if output > self.out_max:
            output = self.out_max
            if abs(self.Ki) > 1e-6:
                self.integral = (output - proportional - derivative) / self.Ki
        elif output < self.out_min:
            output = self.out_min
            if abs(self.Ki) > 1e-6:
                self.integral = (output - proportional - derivative) / self.Ki
                
        return output

class PIDLQRNode(Node):
    def __init__(self):
        super().__init__('pid_lqr_node')
        
        # Load parameters fisik dari config YAML
        self.declare_parameter('config_dir', '')
        config_dir = self.get_parameter('config_dir').value
        if not config_dir:
            # fallback: relative to __file__ (source tree)
            config_dir = os.path.join(os.path.dirname(__file__), '..', 'config')
        config_path = os.path.join(config_dir, 'quadrotor_params.yaml')
        
        if not os.path.exists(config_path):
            self.get_logger().error(f"Config file not found at {config_path}")
            return
            
        with open(config_path, 'r') as f:
            yaml_data = yaml.safe_load(f)
            self.params = yaml_data['physics']
            self.limits = yaml_data['actuator_limits']
            self.act_phys = yaml_data['actuator_physics']
            
        # Dapatkan nilai Gain menggunakan Solver LQR bawaan Anda
        solver = PIDLQRSolver(self.params)
        gains = solver.compute_all_gains()
        
        self.dt = 0.02 # Asumsi 50Hz, akan diupdate dinamis
        
        # Inisialisasi Blok PID untuk seluruh sumbu
        self.pid_x_out = PID(gains['x_outer']['Kp'], gains['x_outer']['Ki'], gains['x_outer']['Kd'], self.dt, -self.limits['angle_max'], self.limits['angle_max'])
        self.pid_x_in  = PID(gains['x_inner']['Kp'], gains['x_inner']['Ki'], gains['x_inner']['Kd'], self.dt, -self.limits['tau_rp_max'], self.limits['tau_rp_max'])
        
        self.pid_y_out = PID(gains['y_outer']['Kp'], gains['y_outer']['Ki'], gains['y_outer']['Kd'], self.dt, -self.limits['angle_max'], self.limits['angle_max'])
        self.pid_y_in  = PID(gains['y_inner']['Kp'], gains['y_inner']['Ki'], gains['y_inner']['Kd'], self.dt, -self.limits['tau_rp_max'], self.limits['tau_rp_max'])
        
        self.pid_z   = PID(gains['z']['Kp'], gains['z']['Ki'], gains['z']['Kd'], self.dt, -self.limits['thrust_max'], self.limits['thrust_max'])
        self.pid_yaw = PID(gains['yaw']['Kp'], gains['yaw']['Ki'], gains['yaw']['Kd'], self.dt, -self.limits['tau_y_max'], self.limits['tau_y_max'])
        
        # Konstanta Fisika dan Matriks Mixer
        self.g = self.params['g']
        self.m = self.params['mass']
        kf, km = self.act_phys['kf'], self.act_phys['km']
        self.w_max, self.w_min = self.act_phys['omega_max'], self.act_phys['omega_min']
        d = self.params['arm_length'] * 0.707106781  # sin(45 deg)
        
        M = np.array([
            [kf, kf, kf, kf],
            [-kf*d, kf*d, kf*d, -kf*d],
            [-kf*d, kf*d, -kf*d, kf*d],
            [-km, -km, km, km]
        ])
        self.M_inv = np.linalg.inv(M)
        
        # Konfigurasi drone_id
        self.declare_parameter('drone_id', 1)
        did = self.get_parameter('drone_id').get_parameter_value().integer_value
        if not did:
            did = 1
        self.drone_id = did

        # Target referensi awal formasi swarm (Z = 2.0m, Y sesuai urutan drone)
        spacing = 2.0
        self.formation_x = 0.0
        self.formation_y = float((self.drone_id - 4.0) * spacing)
        self.formation_z = 2.0
        self.x_cmd, self.y_cmd, self.z_cmd = self.formation_x, self.formation_y, self.formation_z
        self.yaw_cmd = np.radians(0.0)
        self.target_pose_received = False
        # Velocity feedforward dari mid-level ORCA (default nol)
        self.vx_cmd = 0.0
        self.vy_cmd = 0.0
        self.k_ff = 0.07  # Feedforward gain: 1 m/s → 0.07 rad (~4°) tambahan pitch/roll

        # State Pre-filter (Low-Pass Filter) untuk referensi [posisi, kecepatan]
        self.filt_x = [0.0, 0.0]
        self.filt_y = [0.0, 0.0]
        self.filt_z = [0.0, 0.0]
        self.filt_yaw = [0.0, 0.0]
        
        self.w_n_sq = 2.25
        self.two_zeta_wn = 3.0

        # Konfigurasi Log Directory
        self.declare_parameter('log_dir', os.getcwd())
        log_dir = self.get_parameter('log_dir').value
        
        self.subscription = self.create_subscription(Odometry, 'odometry', self.odom_callback, 10)
        self.target_sub = self.create_subscription(PoseStamped, 'target_pose', self.target_pose_callback, 10)
        self.vel_sub = self.create_subscription(TwistStamped, 'target_velocity', self.target_velocity_callback, 10)
        self.publisher = self.create_publisher(Actuators, 'command/motor_speed', 10)
        self.marker_pub = self.create_publisher(MarkerArray, 'marker_visual', 10)
            
        self.get_logger().info("=========================================")
        self.get_logger().info(f"OTAK PID-LQR iris_{did} AKTIF! Misi: Melayang di Z=2.0m")
        self.get_logger().info("=========================================")
        
        self.csv_path = os.path.join(log_dir, f'flight_data_log_lqr_iris_{did}.csv')
        self.csv_file = open(self.csv_path, mode='w', newline='')
        self.csv_writer = csv.writer(self.csv_file)
        self.csv_writer.writerow(['Time_s', 'X', 'Y', 'Z', 'Roll_deg', 'Pitch_deg', 'Yaw_deg',
                                  'Ref_X', 'Ref_Y', 'Ref_Z', 'Ref_Yaw',
                                  'vx', 'vy', 'vz', 'p', 'q', 'r',
                                  'T_pert', 'tau_x', 'tau_y', 'tau_z',
                                  'RPM_0', 'RPM_1', 'RPM_2', 'RPM_3'])
        
        self.start_time = None
        self.last_time = None
        self.last_csv_log_time = 0.0

    def publish_drone_marker(self, x, y, z, roll, pitch, yaw, q_msg):
        color_map = {
            1: (1.0, 0.0, 0.0),
            2: (1.0, 0.5, 0.0),
            3: (1.0, 1.0, 0.0),
            4: (0.0, 1.0, 0.0),
            5: (0.0, 1.0, 1.0),
            6: (0.0, 0.5, 1.0),
            7: (1.0, 0.0, 1.0)
        }
        r_c, g_c, b_c = color_map.get(self.drone_id, (1.0, 1.0, 1.0))
        
        ma = MarkerArray()
        
        # 1. Bodi Drone (Bola / Sphere) saja, sangat simpel!
        m_body = Marker()
        m_body.header.frame_id = 'world'
        m_body.header.stamp = self.get_clock().now().to_msg()
        m_body.ns = 'swarm_drones'
        m_body.id = self.drone_id
        m_body.type = Marker.SPHERE
        m_body.action = Marker.ADD
        m_body.pose.position.x = float(x)
        m_body.pose.position.y = float(y)
        m_body.pose.position.z = float(z)
        m_body.pose.orientation = q_msg
        
        m_body.scale.x = 0.15 # Diameter bodi asli di Gazebo (15cm)
        m_body.scale.y = 0.15
        m_body.scale.z = 0.15
        
        m_body.color.r = float(r_c)
        m_body.color.g = float(g_c)
        m_body.color.b = float(b_c)
        m_body.color.a = 1.0
        
        ma.markers.append(m_body)
        
        self.marker_pub.publish(ma)

    def euler_from_quaternion(self, x, y, z, w):
        t0 = +2.0 * (w * x + y * z)
        t1 = +1.0 - 2.0 * (x * x + y * y)
        roll_x = math.atan2(t0, t1)
        t2 = +2.0 * (w * y - z * x)
        t2 = +1.0 if t2 > +1.0 else t2
        t2 = -1.0 if t2 < -1.0 else t2
        pitch_y = math.asin(t2)
        t3 = +2.0 * (w * z + x * y)
        t4 = +1.0 - 2.0 * (y * y + z * z)
        yaw_z = math.atan2(t3, t4)
        return roll_x, pitch_y, yaw_z

    def odom_callback(self, msg):
        sec = msg.header.stamp.sec
        nanosec = msg.header.stamp.nanosec
        current_time = sec + nanosec * 1e-9
        
        x = msg.pose.pose.position.x
        y = msg.pose.pose.position.y
        z = msg.pose.pose.position.z

        if self.start_time is None:
            self.start_time = current_time
            self.last_time = current_time
            self.spawn_x = x
            self.spawn_y = y
            qx0 = msg.pose.pose.orientation.x
            qy0 = msg.pose.pose.orientation.y
            qz0 = msg.pose.pose.orientation.z
            qw0 = msg.pose.pose.orientation.w
            _, _, yaw0 = self.euler_from_quaternion(qx0, qy0, qz0, qw0)
            self.spawn_yaw = yaw0
            self.yaw_cmd = yaw0
            self.filt_x = [x, 0.0]
            self.filt_y = [y, 0.0]
            self.filt_z = [z, 0.0]
            self.filt_yaw = [yaw0, 0.0]
            return
            
        t = current_time - self.start_time
        dt = current_time - self.last_time
        self.last_time = current_time
        
        if not self.target_pose_received:
            self.x_cmd = self.formation_x
            self.y_cmd = self.formation_y
            if hasattr(self, 'spawn_yaw'):
                self.yaw_cmd = self.spawn_yaw
        
        reset_derivative = False
        if dt <= 0 or dt >= 0.1:
            reset_derivative = True
            dt_control = 0.02
        else:
            dt_control = dt
            
        self.pid_x_out.dt = dt_control
        self.pid_x_in.dt = dt_control
        self.pid_y_out.dt = dt_control
        self.pid_y_in.dt = dt_control
        self.pid_z.dt = dt_control
        self.pid_yaw.dt = dt_control

        if z < 0.15:
            self.pid_x_in.integral = 0.0
            self.pid_y_in.integral = 0.0
            self.pid_yaw.integral = 0.0
        
        x = msg.pose.pose.position.x
        y = msg.pose.pose.position.y
        z = msg.pose.pose.position.z
        
        qx = msg.pose.pose.orientation.x
        qy = msg.pose.pose.orientation.y
        qz = msg.pose.pose.orientation.z
        qw = msg.pose.pose.orientation.w
        phi, theta, yaw = self.euler_from_quaternion(qx, qy, qz, qw)
        
        vx = msg.twist.twist.linear.x
        vy = msg.twist.twist.linear.y
        vz = msg.twist.twist.linear.z
        p = msg.twist.twist.angular.x
        q_ang = msg.twist.twist.angular.y
        r_ang = msg.twist.twist.angular.z
        
        self.filt_x[1] += (self.w_n_sq * (self.x_cmd - self.filt_x[0]) - self.two_zeta_wn * self.filt_x[1]) * dt_control
        self.filt_x[0] += self.filt_x[1] * dt_control
        
        self.filt_y[1] += (self.w_n_sq * (self.y_cmd - self.filt_y[0]) - self.two_zeta_wn * self.filt_y[1]) * dt_control
        self.filt_y[0] += self.filt_y[1] * dt_control
        
        self.filt_z[1] += (self.w_n_sq * (self.z_cmd - self.filt_z[0]) - self.two_zeta_wn * self.filt_z[1]) * dt_control
        self.filt_z[0] += self.filt_z[1] * dt_control
        
        # Yaw: bypass filter second-order — gunakan yaw_cmd langsung agar tidak ada lag ganda
        # (mid-level sudah melakukan smoothing via alpha_yaw)
        yaw_cmd_norm = (self.yaw_cmd + np.pi) % (2 * np.pi) - np.pi
        cos_yaw = math.cos(yaw)
        sin_yaw = math.sin(yaw)

        # Transformasi Error Posisi dari World Frame ke Body Frame
        err_x_world = self.filt_x[0] - x
        err_y_world = self.filt_y[0] - y
        err_x_body =  err_x_world * cos_yaw + err_y_world * sin_yaw
        err_y_body = -err_x_world * sin_yaw + err_y_world * cos_yaw

        # Transformasi Velocity Feedforward dari World Frame ke Body Frame
        vx_body =  self.vx_cmd * cos_yaw + self.vy_cmd * sin_yaw
        vy_body = -self.vx_cmd * sin_yaw + self.vy_cmd * cos_yaw

        # Pitch (theta) mengontrol gerakan Body X, Roll (phi) mengontrol gerakan Body Y
        theta_ref_raw = self.pid_x_out.compute(err_x_body, reset_derivative=reset_derivative) + self.k_ff * vx_body
        max_angle_takeoff = max(math.radians(2.0), self.limits['angle_max'] * min(z / 0.5, 1.0))
        theta_ref = np.clip(theta_ref_raw, -max_angle_takeoff, max_angle_takeoff)
        err_theta = theta_ref - theta
        uy_pid = self.pid_x_in.compute(err_theta, reset_derivative=reset_derivative)
        
        phi_ref_raw = self.pid_y_out.compute(err_y_body, reset_derivative=reset_derivative) + self.k_ff * vy_body
        phi_ref = np.clip(phi_ref_raw, -max_angle_takeoff, max_angle_takeoff)
        err_phi = phi_ref - phi
        ux_pid = self.pid_y_in.compute(err_phi, reset_derivative=reset_derivative)
        
        err_z = self.filt_z[0] - z
        uz_pid = self.pid_z.compute(err_z, reset_derivative=reset_derivative) 
        
        # Normalisasi error yaw ke range [-pi, pi] untuk menghindari loncat 2pi
        err_yaw = (yaw_cmd_norm - yaw + np.pi) % (2 * np.pi) - np.pi
        uyaw_pid = self.pid_yaw.compute(err_yaw, reset_derivative=reset_derivative)
        
        U_cmd = np.array([uz_pid + (self.m * self.g), ux_pid, uy_pid, uyaw_pid])
        
        w_sq_cmd = self.M_inv @ U_cmd
        w_cmd = np.sqrt(np.maximum(w_sq_cmd, 0)) 
        w_cmd = np.clip(w_cmd, self.w_min, self.w_max)
        
        act_msg = Actuators()
        act_msg.velocity = [float(w_cmd[0]), float(w_cmd[1]), float(w_cmd[2]), float(w_cmd[3])]
        act_msg.normalized = act_msg.velocity
        self.publisher.publish(act_msg)
        
        roll_deg = math.degrees(phi)
        pitch_deg = math.degrees(theta)
        yaw_deg = math.degrees(yaw)
        
        if int(t * 50) % 15 == 0:
            self.get_logger().info(
                f"\n"
                f"━━━━━━━━━━━━━━━━━━━ [PID-LQR | T={t:.1f}s] ━━━━━━━━━━━━━━━━━━━\n"
                f"  Posisi  │  Aktual   │  Target   │  Error\n"
                f"  X       │  {x:+7.3f}m │  {self.x_cmd:+7.3f}m │  {self.x_cmd - x:+7.3f}m\n"
                f"  Y       │  {y:+7.3f}m │  {self.y_cmd:+7.3f}m │  {self.y_cmd - y:+7.3f}m\n"
                f"  Z       │  {z:+7.3f}m │  {self.z_cmd:+7.3f}m │  {self.z_cmd - z:+7.3f}m\n"
                f"  Yaw     │  {yaw_deg:+7.2f}° │  {math.degrees(self.yaw_cmd):+7.2f}° │  {math.degrees(self.yaw_cmd) - yaw_deg:+7.2f}°\n"
                f"  RPM → [{int(w_cmd[0])}, {int(w_cmd[1])}, {int(w_cmd[2])}, {int(w_cmd[3])}]\n"
                f"━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━"
            )
        if t - self.last_csv_log_time >= 0.05:  # 20 Hz
            self.csv_writer.writerow([t, x, y, z, roll_deg, pitch_deg, yaw_deg,
                                      self.filt_x[0], self.filt_y[0], self.filt_z[0], math.degrees(yaw_cmd_norm),
                                      vx, vy, vz, p, q_ang, r_ang,
                                      uz_pid, ux_pid, uy_pid, uyaw_pid,
                                      w_cmd[0], w_cmd[1], w_cmd[2], w_cmd[3]])
            self.last_csv_log_time = t
        self.publish_drone_marker(x, y, z, phi, theta, yaw, msg.pose.pose.orientation)

    def target_pose_callback(self, msg):
        self.x_cmd = msg.pose.position.x
        self.y_cmd = msg.pose.position.y
        self.z_cmd = msg.pose.position.z
        self.target_pose_received = True

        # Extract yaw from orientation quaternion sent by mid-level ORCA
        qx = msg.pose.orientation.x
        qy = msg.pose.orientation.y
        qz = msg.pose.orientation.z
        qw = msg.pose.orientation.w
        if abs(qw) < 0.9999 or abs(qz) > 1e-6:
            _, _, yaw_target = self.euler_from_quaternion(qx, qy, qz, qw)
            self.yaw_cmd = yaw_target

    def target_velocity_callback(self, msg):
        """Terima kecepatan ORCA dari mid-level sebagai velocity feedforward."""
        self.vx_cmd = float(msg.twist.linear.x)
        self.vy_cmd = float(msg.twist.linear.y)

    def destroy_node(self):
        self.csv_file.close()
        super().destroy_node()

def main(args=None):
    import sys
    rclpy.init(args=args)
    node = PIDLQRNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info('Shutting down gracefully...')
    finally:
        node.destroy_node()
        try:
            rclpy.shutdown()
        except Exception:
            pass
        sys.exit(0)

if __name__ == '__main__':
    main()
