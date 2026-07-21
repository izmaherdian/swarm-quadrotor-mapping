import rclpy
from rclpy.node import Node
from nav_msgs.msg import Odometry
from actuator_msgs.msg import Actuators
from geometry_msgs.msg import PoseStamped
import math
import csv
import os
import numpy as np
import yaml
from ament_index_python.packages import get_package_share_directory

try:
    from swarm_low_level.solver_pid_lqr import PIDLQRSolver
except ImportError:
    from solver_pid_lqr import PIDLQRSolver

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
        # Deklarasi drone_id SEBELUM super().__init__ agar tersedia saat inisialisasi
        # Tidak bisa, jadi deklarasikan dulu lalu baca setelah super().__init__
        super().__init__('pid_lqr_node')
        
        # ── Parameter drone_id ────────────────────────────────────────────────
        # KUNCI: gunakan drone_id untuk membangun nama topic secara dinamis.
        # Ini lebih andal daripada remapping launch file untuk absolute topic names.
        self.declare_parameter('drone_id', 1)
        self._drone_id = self.get_parameter('drone_id').value
        
        # Membaca parameter quadrotor dari quadrotor_params.yaml dengan fallback aman
        try:
            from ament_index_python.packages import get_package_share_directory
            pkg_share = get_package_share_directory('swarm_low_level')
            config_path = os.path.join(pkg_share, 'config', 'quadrotor_params.yaml')
            if not os.path.exists(config_path):
                raise FileNotFoundError(f"{config_path} not found")
        except Exception:
            curr_dir = os.path.dirname(os.path.abspath(__file__))
            config_path = os.path.abspath(os.path.join(curr_dir, '..', 'config', 'quadrotor_params.yaml'))
            if not os.path.exists(config_path):
                config_path = "/mnt/windows/Izma_S2_InstrumentasiKontrol_ITB/Akademik/Engineering Physics International Conference/#2 Try/swarm_ws/src/swarm_low_level/config/quadrotor_params.yaml"
        
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
        
        self.pid_y_out = PID(abs(gains['y_outer']['Kp']), abs(gains['y_outer']['Ki']), abs(gains['y_outer']['Kd']), self.dt, -self.limits['angle_max'], self.limits['angle_max'])
        self.pid_y_in  = PID(gains['y_inner']['Kp'], gains['y_inner']['Ki'], gains['y_inner']['Kd'], self.dt, -self.limits['tau_rp_max'], self.limits['tau_rp_max'])
        
        self.pid_z   = PID(gains['z']['Kp'], gains['z']['Ki'], gains['z']['Kd'], self.dt, -self.limits['thrust_max'], self.limits['thrust_max'])
        self.pid_yaw = PID(gains['yaw']['Kp'], gains['yaw']['Ki'], gains['yaw']['Kd'], self.dt, -self.limits['tau_y_max'], self.limits['tau_y_max'])
        
        # Konstanta Fisika dan Matriks Mixer
        self.g = self.params['g']
        self.m = self.params['mass']
        kf, km = self.act_phys['kf'], self.act_phys['km']
        self.w_max, self.w_min = self.act_phys['omega_max'], self.act_phys['omega_min']
        d = 0.15  # Jarak eksak rotor dari pusat (model.sdf: x=±0.15, y=±0.15)
        
        M = np.array([
            [kf, kf, kf, kf],
            [kf*d, -kf*d, -kf*d, kf*d],    # tau_x (Roll)
            [kf*d, -kf*d, kf*d, -kf*d],    # tau_y (Pitch)
            [-km, -km, km, km]             # tau_z (Yaw)
        ])
        self.M_inv = np.linalg.inv(M)
        
        # Target referensi awal di (0, 0, 2)
        self.x_cmd, self.y_cmd, self.z_cmd = 0.0, 0.0, 2.0
        self.yaw_cmd = np.radians(0.0)
        
        # State Pre-filter (Low-Pass Filter) untuk referensi [posisi, kecepatan]
        # Mulai dari titik awal (0, 0, 0)
        self.filt_x = [0.0, 0.0]
        self.filt_y = [0.0, 0.0]
        self.filt_z = [0.0, 0.0]
        self.filt_yaw = [0.0, 0.0]
        
        # Parameter Pre-filter (wn = 1.5 rad/s, zeta = 1.0)
        self.w_n_sq = 2.25
        self.two_zeta_wn = 3.0
        
        # ── Topic Names berdasarkan drone_id ─────────────────────────────────
        # Setiap controller subscribe/publish ke topicnya sendiri secara eksplisit.
        # Ini menghindari ketergantungan pada remapping launch file.
        did = self._drone_id
        odom_topic   = f'/model/iris_{did}/odometry'
        target_topic = f'/iris_{did}/target_pose'
        motor_topic  = f'/model/iris_{did}/command/motor_speed'
        
        # Konfigurasi Log Directory
        self.declare_parameter('log_dir', os.getcwd())
        log_dir = self.get_parameter('log_dir').value
        
        self.subscription = self.create_subscription(Odometry,    odom_topic,   self.odom_callback,         10)
        self.target_sub   = self.create_subscription(PoseStamped, target_topic, self.target_pose_callback,  10)
        self.publisher    = self.create_publisher(Actuators,       motor_topic,  10)
            
        self.get_logger().info(f"=========================================")
        self.get_logger().info(f"OTAK PID-LQR AKTIF! iris_{did} | Misi: Melayang di Z=2.0m")
        self.get_logger().info(f"  Odom  : {odom_topic}")
        self.get_logger().info(f"  Motor : {motor_topic}")
        self.get_logger().info(f"=========================================")
        
        self.csv_path = os.path.join(log_dir, 'flight_data_log_lqr.csv')
        self.csv_file = open(self.csv_path, mode='w', newline='')
        self.csv_writer = csv.writer(self.csv_file)
        self.csv_writer.writerow(['Time_s', 'X', 'Y', 'Z', 'Roll_deg', 'Pitch_deg', 'Yaw_deg',
                                  'Ref_X', 'Ref_Y', 'Ref_Z', 'Ref_Yaw',
                                  'vx', 'vy', 'vz', 'p', 'q', 'r',
                                  'T_pert', 'tau_x', 'tau_y', 'tau_z',
                                  'RPM_0', 'RPM_1', 'RPM_2', 'RPM_3'])
        
        self.start_time = None
        self.last_time = None

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
        
        if self.start_time is None:
            self.start_time = current_time
            self.last_time = current_time
            return
            
        t = current_time - self.start_time
        dt = current_time - self.last_time
        self.last_time = current_time
        
        # Tangani Time Jumps atau Nilai dt Invalid
        reset_derivative = False
        if dt <= 0 or dt >= 0.1:
            reset_derivative = True
            dt_control = 0.02  # Gunakan default dt untuk menjaga stabilitas filter/PID
        else:
            dt_control = dt
            
        # Update dt Dinamis ke seluruh PID
        self.pid_x_out.dt = dt_control
        self.pid_x_in.dt = dt_control
        self.pid_y_out.dt = dt_control
        self.pid_y_in.dt = dt_control
        self.pid_z.dt = dt_control
        self.pid_yaw.dt = dt_control
        
        # 1. BACA SENSOR
        x = msg.pose.pose.position.x
        y = msg.pose.pose.position.y
        z = msg.pose.pose.position.z

        qx = msg.pose.pose.orientation.x
        qy = msg.pose.pose.orientation.y
        qz = msg.pose.pose.orientation.z
        qw = msg.pose.pose.orientation.w
        phi, theta, yaw = self.euler_from_quaternion(qx, qy, qz, qw)

        # Kunci posisi awal X dan Y dari sensor odometry saat startup
        if not getattr(self, '_initial_pos_set', False):
            self.x_cmd = x
            self.y_cmd = y
            self.z_cmd = 2.0
            self.yaw_cmd = yaw
            self.filt_x = [x, 0.0]
            self.filt_y = [y, 0.0]
            self.filt_z = [z, 0.0]
            self.filt_yaw = [yaw, 0.0]
            self._initial_pos_set = True
        
        # Kecepatan Linear dan Angular
        vx = msg.twist.twist.linear.x
        vy = msg.twist.twist.linear.y
        vz = msg.twist.twist.linear.z
        p = msg.twist.twist.angular.x
        q_ang = msg.twist.twist.angular.y
        r_ang = msg.twist.twist.angular.z
        
        # 1.5 UPDATE PRE-FILTER
        # Mencegah step response mendadak dengan menghaluskan target (seperti di simulator)
        # Gunakan dt_control untuk stabilitas integrasi numerik pre-filter
        self.filt_x[1] += (self.w_n_sq * (self.x_cmd - self.filt_x[0]) - self.two_zeta_wn * self.filt_x[1]) * dt_control
        self.filt_x[0] += self.filt_x[1] * dt_control
        
        self.filt_y[1] += (self.w_n_sq * (self.y_cmd - self.filt_y[0]) - self.two_zeta_wn * self.filt_y[1]) * dt_control
        self.filt_y[0] += self.filt_y[1] * dt_control
        
        self.filt_z[1] += (self.w_n_sq * (self.z_cmd - self.filt_z[0]) - self.two_zeta_wn * self.filt_z[1]) * dt_control
        self.filt_z[1] = np.clip(self.filt_z[1], -0.5, 0.5) # Max 0.5 m/s climb rate
        self.filt_z[0] += self.filt_z[1] * dt_control
        
        self.filt_yaw[1] += (self.w_n_sq * (self.yaw_cmd - self.filt_yaw[0]) - self.two_zeta_wn * self.filt_yaw[1]) * dt_control
        self.filt_yaw[0] += self.filt_yaw[1] * dt_control
        
        # 2. PROSES DI OTAK (KONTROLER) menggunakan target ber-filter
        err_x = self.filt_x[0] - x
        theta_ref = self.pid_x_out.compute(err_x, reset_derivative=reset_derivative)
        
        err_y = self.filt_y[0] - y
        phi_ref = self.pid_y_out.compute(err_y, reset_derivative=reset_derivative)

        # Ground Anti-Saturation: Saat masih di darat (z < 0.15m), kunci moment = 0.0
        # agar ke-4 motor berputar pada RPM yang sama persis untuk lift-off tegak lurus
        if z < 0.15:
            theta_ref = 0.0
            phi_ref   = 0.0
            self.pid_x_out.integral = 0.0
            self.pid_y_out.integral = 0.0
            self.pid_x_in.integral  = 0.0
            self.pid_y_in.integral  = 0.0

        err_theta = theta_ref - theta
        uy_pid = self.pid_x_in.compute(err_theta, reset_derivative=reset_derivative)
        
        err_phi = phi_ref - phi
        ux_pid = - self.pid_y_in.compute(err_phi, reset_derivative=reset_derivative)
        
        err_z = self.filt_z[0] - z
        uz_pid = self.pid_z.compute(err_z, reset_derivative=reset_derivative) 
        
        err_yaw = self.filt_yaw[0] - yaw
        uyaw_pid = self.pid_yaw.compute(err_yaw, reset_derivative=reset_derivative)

        if z < 0.15:
            ux_pid = 0.0
            uy_pid = 0.0
            uyaw_pid = 0.0
        
        # 3. KIRIM PERINTAH KE OTOT (AKTUATOR)
        # Menambah gaya berat agar hovering, lalu dikonversi ke kecepatan rotasi via Inverse Mixer
        U_cmd = np.array([uz_pid + (self.m * self.g), ux_pid, uy_pid, uyaw_pid])
        
        w_sq_cmd = self.M_inv @ U_cmd
        w_cmd = np.sqrt(np.maximum(w_sq_cmd, 0))
        # Convert rad/s to RPM for Gazebo MulticopterMotorModel plugin (1 rad/s = 9.5492968 RPM)
        RAD2RPM = 9.549296585513721
        w_rpm = w_cmd * RAD2RPM
        
        act_msg = Actuators()
        act_msg.velocity = [float(w_rpm[0]), float(w_rpm[1]), float(w_rpm[2]), float(w_rpm[3])]
        act_msg.normalized = act_msg.velocity  # Gazebo bridge trick
        self.publisher.publish(act_msg)
        
        # Logging
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

    def target_pose_callback(self, msg):
        # Misi Lepas Landas / Hold Ketinggian:
        # Hanya mengupdate target Z (misal Z=2.0m),
        # X_cmd dan Y_cmd tetap terkunci di posisi awal spawn masing-masing drone
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
