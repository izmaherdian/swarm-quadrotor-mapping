import rclpy
from rclpy.node import Node
from nav_msgs.msg import Odometry
from actuator_msgs.msg import Actuators
from geometry_msgs.msg import PoseStamped, Point as GeometryPoint
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
        
        output_no_i = proportional + derivative
        if not ((output_no_i > self.out_max and error > 0) or (output_no_i < self.out_min and error < 0)):
            self.integral += error * self.dt
            
        output = proportional + self.Ki * self.integral + derivative
        return np.clip(output, self.out_min, self.out_max)

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
        
        # 1. Kotak 3D (Cube)
        m_cube = Marker()
        m_cube.header.frame_id = 'world'
        m_cube.header.stamp = self.get_clock().now().to_msg()
        m_cube.ns = f'drone_cube_{self.drone_id}'
        m_cube.id = 0
        m_cube.type = Marker.CUBE
        m_cube.action = Marker.ADD
        m_cube.pose.position.x = float(x)
        m_cube.pose.position.y = float(y)
        m_cube.pose.position.z = float(z)
        m_cube.pose.orientation = q_msg
        m_cube.scale.x = 0.4
        m_cube.scale.y = 0.4
        m_cube.scale.z = 0.15
        m_cube.color.r = float(r_c)
        m_cube.color.g = float(g_c)
        m_cube.color.b = float(b_c)
        m_cube.color.a = 0.9
        ma.markers.append(m_cube)
        
        # 2. Sumbu X (Depan) - Merah (Panah)
        x_dir_x = math.cos(yaw) * math.cos(pitch)
        x_dir_y = math.sin(yaw) * math.cos(pitch)
        x_dir_z = math.sin(pitch)
        
        m_x = Marker()
        m_x.header.frame_id = 'world'
        m_x.header.stamp = m_cube.header.stamp
        m_x.ns = f'axis_x_{self.drone_id}'
        m_x.id = 1
        m_x.type = Marker.ARROW
        m_x.action = Marker.ADD
        m_x.scale.x = 0.04
        m_x.scale.y = 0.08
        m_x.scale.z = 0.1
        m_x.color.r, m_x.color.g, m_x.color.b, m_x.color.a = 1.0, 0.0, 0.0, 1.0
        
        p1 = GeometryPoint(x=float(x), y=float(y), z=float(z))
        p2_x = GeometryPoint(x=float(x + 0.6 * x_dir_x), y=float(y + 0.6 * x_dir_y), z=float(z + 0.6 * x_dir_z))
        m_x.points = [p1, p2_x]
        ma.markers.append(m_x)
        
        # 3. Sumbu Y (Samping Kiri) - Hijau (Panah) - HANYA X dan Y (Tanpa Sumbu Z!)
        y_dir_x = -math.sin(yaw)
        y_dir_y = math.cos(yaw)
        y_dir_z = 0.0
        
        m_y = Marker()
        m_y.header.frame_id = 'world'
        m_y.header.stamp = m_cube.header.stamp
        m_y.ns = f'axis_y_{self.drone_id}'
        m_y.id = 2
        m_y.type = Marker.ARROW
        m_y.action = Marker.ADD
        m_y.scale.x = 0.04
        m_y.scale.y = 0.08
        m_y.scale.z = 0.1
        m_y.color.r, m_y.color.g, m_y.color.b, m_y.color.a = 0.0, 1.0, 0.0, 1.0
        
        p2_y = GeometryPoint(x=float(x + 0.6 * y_dir_x), y=float(y + 0.6 * y_dir_y), z=float(z + 0.6 * y_dir_z))
        m_y.points = [p1, p2_y]
        ma.markers.append(m_y)
        
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
            self.filt_x = [x, 0.0]
            self.filt_y = [y, 0.0]
            self.filt_z = [z, 0.0]
            self.filt_yaw = [yaw0, 0.0]
            return
            
        t = current_time - self.start_time
        dt = current_time - self.last_time
        self.last_time = current_time
        
        if z < 1.5 and not self.target_pose_received:
            self.x_cmd = self.formation_x
            self.y_cmd = self.formation_y
        
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
        
        self.filt_yaw[1] += (self.w_n_sq * (self.yaw_cmd - self.filt_yaw[0]) - self.two_zeta_wn * self.filt_yaw[1]) * dt_control
        self.filt_yaw[0] += self.filt_yaw[1] * dt_control
        
        err_x = self.filt_x[0] - x
        theta_ref_raw = self.pid_x_out.compute(err_x, reset_derivative=reset_derivative)
        max_angle_takeoff = max(math.radians(2.0), self.limits['angle_max'] * min(z / 0.5, 1.0))
        theta_ref = np.clip(theta_ref_raw, -max_angle_takeoff, max_angle_takeoff)
        err_theta = theta_ref - theta
        uy_pid = self.pid_x_in.compute(err_theta, reset_derivative=reset_derivative)
        
        err_y = self.filt_y[0] - y
        phi_ref_raw = self.pid_y_out.compute(err_y, reset_derivative=reset_derivative)
        phi_ref = np.clip(phi_ref_raw, -max_angle_takeoff, max_angle_takeoff)
        err_phi = phi_ref - phi
        ux_pid = self.pid_y_in.compute(err_phi, reset_derivative=reset_derivative)
        
        err_z = self.filt_z[0] - z
        uz_pid = self.pid_z.compute(err_z, reset_derivative=reset_derivative) 
        
        err_yaw = self.filt_yaw[0] - yaw
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
            
        self.csv_writer.writerow([t, x, y, z, roll_deg, pitch_deg, yaw_deg,
                                  self.filt_x[0], self.filt_y[0], self.filt_z[0], self.filt_yaw[0],
                                  vx, vy, vz, p, q_ang, r_ang,
                                  uz_pid, ux_pid, uy_pid, uyaw_pid,
                                  w_cmd[0], w_cmd[1], w_cmd[2], w_cmd[3]])
        self.csv_file.flush()
        
        self.publish_drone_marker(x, y, z, phi, theta, yaw, msg.pose.pose.orientation)

    def target_pose_callback(self, msg):
        self.x_cmd = msg.pose.position.x
        self.y_cmd = msg.pose.position.y
        self.z_cmd = msg.pose.position.z

    def destroy_node(self):
        self.csv_file.close()
        super().destroy_node()

def main(args=None):
    rclpy.init(args=args)
    node = PIDLQRNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
