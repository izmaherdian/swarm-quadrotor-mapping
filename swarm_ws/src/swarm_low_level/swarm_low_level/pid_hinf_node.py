import rclpy
from rclpy.node import Node
from nav_msgs.msg import Odometry, Path
from actuator_msgs.msg import Actuators
from geometry_msgs.msg import PoseStamped, Point as GeometryPoint, TwistStamped, Twist
from visualization_msgs.msg import Marker, MarkerArray
import math
import csv
import os
import numpy as np
import yaml
from .solver_pid_hinf import PIDHinfSolver

class PID:
    def __init__(self, Kp, Ki, Kd, dt, out_min=-np.inf, out_max=np.inf, i_max=np.inf):
        self.Kp = Kp
        self.Ki = Ki
        self.Kd = Kd
        self.dt = dt
        self.out_min = out_min
        self.out_max = out_max
        self.i_max = i_max
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
        
        # 1. Hitung output tanpa kontribusi integral baru terlebih dahulu
        output_no_i = proportional + self.Ki * self.integral + derivative
        
        # 2. Hanya integrasikan error jika output tidak jenuh, atau jika integrasi mengurangi kejenuhan
        integrate = True
        if output_no_i > self.out_max and error > 0:
            integrate = False
        elif output_no_i < self.out_min and error < 0:
            integrate = False
            
        if integrate:
            self.integral += error * self.dt
            if self.i_max < np.inf:
                self.integral = float(np.clip(self.integral, -self.i_max, self.i_max))
            
        output = proportional + self.Ki * self.integral + derivative
        
        # 3. Batasi output ke range [out_min, out_max]
        output = np.clip(output, self.out_min, self.out_max)
                
        return output

class PIDHinfNode(Node):
    def __init__(self):
        super().__init__('pid_hinf_node')
        
        # Load parameters fisik dari config YAML
        self.declare_parameter('config_dir', '')
        config_dir = self.get_parameter('config_dir').value
        if not config_dir:
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
            
        # 1. Inisialisasi Solver H-Infinity
        self.solver = PIDHinfSolver(self.params)
        gains = self.solver.get_all_gains()
        
        self.dt = 0.02 # Asumsi 50Hz, akan diupdate dinamis
        
        # Inisialisasi Blok PID untuk seluruh sumbu
        # i_max untuk outer loop dibatasi ke 0.052 rad (3.0 deg) agar integral tidak mendominasi pengereman
        i_max_xy_out = 0.052 / max(abs(gains['x_outer']['Ki']), 1e-3)
        self.pid_x_out = PID(gains['x_outer']['Kp'], gains['x_outer']['Ki'], gains['x_outer']['Kd'], self.dt, -self.limits['angle_max'], self.limits['angle_max'], i_max=i_max_xy_out)
        self.pid_x_in  = PID(gains['x_inner']['Kp'], 0.0, gains['x_inner']['Kd'], self.dt, -self.limits['tau_rp_max'], self.limits['tau_rp_max'])
        
        self.pid_y_out = PID(gains['y_outer']['Kp'], gains['y_outer']['Ki'], gains['y_outer']['Kd'], self.dt, -self.limits['angle_max'], self.limits['angle_max'], i_max=i_max_xy_out)
        self.pid_y_in  = PID(gains['y_inner']['Kp'], 0.0, gains['y_inner']['Kd'], self.dt, -self.limits['tau_rp_max'], self.limits['tau_rp_max'])
        
        self.pid_z   = PID(gains['z']['Kp'], gains['z']['Ki'], gains['z']['Kd'], self.dt, -self.limits['thrust_max'], self.limits['thrust_max'], i_max=2.0)
        self.pid_yaw = PID(gains['yaw']['Kp'] * 1.0, 0.0, gains['yaw']['Kd'], self.dt, -self.limits['tau_y_max'], self.limits['tau_y_max'])
        
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
        # Velocity feedforward dari mid-level ORCA
        self.vx_cmd = 0.0
        self.vy_cmd = 0.0
        self.k_ff = 0.15  # Feedforward gain: 1 m/s → 0.15 rad (~8.6°) pitch/roll untuk tracking gesit
        self.yaw_rate_cmd = 0.0
        self.k_ff_yaw = 0.5 * self.params['iz']  # Feedforward gain: yaw_rate → torque (Nm/(rad/s))

        # State Pre-filter (Low-Pass Filter) untuk referensi [posisi, kecepatan]
        self.filt_x = [0.0, 0.0]
        self.filt_y = [0.0, 0.0]
        self.filt_z = [0.0, 0.0]
        self.filt_yaw = [0.0, 0.0]
        
        self.w_n_sq = 64.0       # omega_n = 8.0 rad/s (responsif cepat, meminimalkan lag pelacakan kecepatan)
        self.two_zeta_wn = 15.2  # 2 * 0.95 * 8.0

        # Konfigurasi Log Directory
        self.declare_parameter('log_dir', os.getcwd())
        log_dir = self.get_parameter('log_dir').value
        
        self.subscription = self.create_subscription(Odometry, 'odometry', self.odom_callback, 10)
        self.target_sub = self.create_subscription(PoseStamped, 'target_pose', self.target_pose_callback, 10)
        self.vel_sub = self.create_subscription(TwistStamped, 'target_velocity', self.target_velocity_callback, 10)
        self.cmd_vel_sub = self.create_subscription(Twist, 'cmd_vel', self.cmd_vel_callback, 10)
        self.publisher = self.create_publisher(Actuators, 'command/motor_speed', 10)
        self.marker_pub = self.create_publisher(MarkerArray, 'marker_visual', 10)
        self.path_pub = self.create_publisher(Path, 'actual_path', 10)
            
        self.get_logger().info("=========================================")
        self.get_logger().info(f"OTAK PID-HINF iris_{did} AKTIF! Misi: Melayang di Z=2.0m")
        self.get_logger().info("=========================================")
        
        self.csv_path = os.path.join(log_dir, f'flight_data_log_hinf_iris_{did}.csv')
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

    def publish_drone_marker(self, x, y, z, roll, pitch, yaw, q_msg, vx=0.0, vy=0.0, vz=0.0):
        color_map = {
            1: (1.0, 0.15, 0.15),  # Iris 1: Vibrant Red
            2: (1.0, 0.60, 0.0),   # Iris 2: Vibrant Orange
            3: (1.0, 0.95, 0.1),   # Iris 3: Yellow
            4: (0.1, 0.95, 0.2),   # Iris 4: Green
            5: (0.1, 0.85, 1.0),   # Iris 5: Cyan
            6: (0.3, 0.45, 1.0),   # Iris 6: Blue
            7: (0.9, 0.20, 1.0)    # Iris 7: Purple
        }
        r_c, g_c, b_c = color_map.get(self.drone_id, (1.0, 1.0, 1.0))
        now_msg = self.get_clock().now().to_msg()
        
        ma = MarkerArray()
        
        # 1. Bodi Drone (Solid Sphere)
        m_body = Marker()
        m_body.header.frame_id = 'world'
        m_body.header.stamp = now_msg
        m_body.ns = f'drone_{self.drone_id}_body'
        m_body.id = 0
        m_body.type = Marker.SPHERE
        m_body.action = Marker.ADD
        m_body.pose.position.x = float(x)
        m_body.pose.position.y = float(y)
        m_body.pose.position.z = float(z)
        m_body.pose.orientation = q_msg
        m_body.scale.x = 0.20
        m_body.scale.y = 0.20
        m_body.scale.z = 0.20
        m_body.color.r = float(r_c)
        m_body.color.g = float(g_c)
        m_body.color.b = float(b_c)
        m_body.color.a = 1.0
        ma.markers.append(m_body)

        # 2. Safety Bubble Clearance (Translucent Sphere r = 0.40m)
        m_bubble = Marker()
        m_bubble.header.frame_id = 'world'
        m_bubble.header.stamp = now_msg
        m_bubble.ns = f'drone_{self.drone_id}_bubble'
        m_bubble.id = 1
        m_bubble.type = Marker.SPHERE
        m_bubble.action = Marker.ADD
        m_bubble.pose.position.x = float(x)
        m_bubble.pose.position.y = float(y)
        m_bubble.pose.position.z = float(z)
        m_bubble.scale.x = 1.10  # diameter 2 * 0.55m
        m_bubble.scale.y = 1.10
        m_bubble.scale.z = 1.10
        m_bubble.color.r = float(r_c)
        m_bubble.color.g = float(g_c)
        m_bubble.color.b = float(b_c)
        m_bubble.color.a = 0.22
        ma.markers.append(m_bubble)

        # 3. Horizon Prediksi Gerak 2 Detik ke Depan di Bidang X-Y (World Frame)
        # Transformasi kecepatan dari Body Frame ke World Frame:
        cos_yaw = math.cos(yaw)
        sin_yaw = math.sin(yaw)
        vx_world = vx * cos_yaw - vy * sin_yaw
        vy_world = vx * sin_yaw + vy * cos_yaw
        speed_2d = math.sqrt(vx_world**2 + vy_world**2)

        tau = 2.0  # Horizon lookahead 2.0 detik (sesuai parameter tau ORCA)
        p_start = GeometryPoint(x=float(x), y=float(y), z=float(z + 0.05))
        if speed_2d > 0.08:
            p_end = GeometryPoint(
                x=float(x + vx_world * tau),
                y=float(y + vy_world * tau),
                z=float(z + 0.05)
            )
        else:
            # Jika sedang hover diam, tunjukkan orientasi hidung drone sepanjang 0.35m
            p_end = GeometryPoint(
                x=float(x + 0.35 * cos_yaw),
                y=float(y + 0.35 * sin_yaw),
                z=float(z + 0.05)
            )

        m_arrow = Marker()
        m_arrow.header.frame_id = 'world'
        m_arrow.header.stamp = now_msg
        m_arrow.ns = f'drone_{self.drone_id}_horizon'
        m_arrow.id = 2
        m_arrow.type = Marker.ARROW
        m_arrow.action = Marker.ADD
        m_arrow.points = [p_start, p_end]
        m_arrow.scale.x = 0.04  # Diameter batang panah
        m_arrow.scale.y = 0.10  # Diameter kepala panah
        m_arrow.scale.z = 0.12  # Panjang kepala panah
        m_arrow.color.r = float(r_c)
        m_arrow.color.g = float(g_c)
        m_arrow.color.b = float(b_c)
        m_arrow.color.a = 0.95
        ma.markers.append(m_arrow)
        
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
            self.formation_x = x
            self.formation_y = y
            self.x_cmd = x
            self.y_cmd = y
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
            self.pid_x_out.integral = 0.0
            self.pid_y_out.integral = 0.0
            self.pid_x_in.integral = 0.0
            self.pid_y_in.integral = 0.0
            self.pid_z.integral = 0.0
            self.pid_yaw.integral = 0.0
        
        x = msg.pose.pose.position.x
        y = msg.pose.pose.position.y
        z = msg.pose.pose.position.z
        
        qx = msg.pose.pose.orientation.x
        qy = msg.pose.pose.orientation.y
        qz = msg.pose.pose.orientation.z
        qw = msg.pose.pose.orientation.w
        phi, theta, yaw = self.euler_from_quaternion(qx, qy, qz, qw)
        self.last_yaw = yaw
        
        vx = msg.twist.twist.linear.x
        vy = msg.twist.twist.linear.y
        vz = msg.twist.twist.linear.z
        p = msg.twist.twist.angular.x
        q_ang = msg.twist.twist.angular.y
        r_ang = msg.twist.twist.angular.z

        # 0. Smooth Takeoff Vertical Ramping (Maksimal 1.0 m/s untuk mencegah lonjakan thrust dan overshoot)
        MAX_CLIMB_RATE = 1.0  # m/s
        if not hasattr(self, 'current_z_target'):
            self.current_z_target = 0.05
        dz = np.clip(self.z_cmd - self.current_z_target, -MAX_CLIMB_RATE * dt_control, MAX_CLIMB_RATE * dt_control)
        self.current_z_target += dz

        self.filt_x[1] += (self.w_n_sq * (self.x_cmd - self.filt_x[0]) - self.two_zeta_wn * self.filt_x[1]) * dt_control
        self.filt_x[0] += self.filt_x[1] * dt_control
        
        self.filt_y[1] += (self.w_n_sq * (self.y_cmd - self.filt_y[0]) - self.two_zeta_wn * self.filt_y[1]) * dt_control
        self.filt_y[0] += self.filt_y[1] * dt_control
        
        self.filt_z[1] += (self.w_n_sq * (self.current_z_target - self.filt_z[0]) - self.two_zeta_wn * self.filt_z[1]) * dt_control
        self.filt_z[0] += self.filt_z[1] * dt_control
        
        # Yaw: bypass filter second-order — gunakan yaw_cmd langsung agar tidak ada lag ganda
        yaw_cmd_norm = (self.yaw_cmd + np.pi) % (2 * np.pi) - np.pi
        cos_yaw = math.cos(yaw)
        sin_yaw = math.sin(yaw)

        # Transformasi Error Posisi dari World Frame ke Body Frame
        err_x_world = self.filt_x[0] - x
        err_y_world = self.filt_y[0] - y
        err_x_body =  err_x_world * cos_yaw + err_y_world * sin_yaw
        err_y_body = -err_x_world * sin_yaw + err_y_world * cos_yaw

        # Kecepatan Linier Aktual dari Sensor Odometry (Twist is ALREADY in Body Frame) — Zero Phase Lag
        vx_body_meas = vx
        vy_body_meas = vy

        # Transformasi Velocity Feedforward dari World Frame ke Body Frame
        vx_body =  self.vx_cmd * cos_yaw + self.vy_cmd * sin_yaw
        vy_body = -self.vx_cmd * sin_yaw + self.vy_cmd * cos_yaw

        # Frame-Invariant Vector Integrator (World Frame Integration -> Body Frame Projection)
        if not hasattr(self, 'integral_x_world'):
            self.integral_x_world = 0.0
            self.integral_y_world = 0.0

        if z >= 0.15 and abs(err_x_world) < 0.25 and abs(err_y_world) < 0.25:
            self.integral_x_world += err_x_world * dt_control
            self.integral_y_world += err_y_world * dt_control
            self.integral_x_world = float(np.clip(self.integral_x_world, -0.4, 0.4))
            self.integral_y_world = float(np.clip(self.integral_y_world, -0.4, 0.4))
        else:
            self.integral_x_world = 0.0
            self.integral_y_world = 0.0

        integral_x_body =  self.integral_x_world * cos_yaw + self.integral_y_world * sin_yaw
        integral_y_body = -self.integral_x_world * sin_yaw + self.integral_y_world * cos_yaw

        i_term_x = self.pid_x_out.Ki * integral_x_body
        i_term_y = self.pid_y_out.Ki * integral_y_body

        # Pitch (theta) Outer Loop — Derivative on Measurement
        theta_ref_raw = (self.pid_x_out.Kp * err_x_body + i_term_x - self.pid_x_out.Kd * vx_body_meas) + self.k_ff * vx_body

        max_angle_takeoff = max(math.radians(2.0), self.limits['angle_max'] * min(z / 0.5, 1.0))
        theta_ref = np.clip(theta_ref_raw, -max_angle_takeoff, max_angle_takeoff)

        # Roll (phi) Outer Loop — Derivative on Measurement
        phi_ref_raw = (self.pid_y_out.Kp * err_y_body + i_term_y - self.pid_y_out.Kd * vy_body_meas) - self.k_ff * vy_body

        phi_ref = np.clip(phi_ref_raw, -max_angle_takeoff, max_angle_takeoff)

        # 1. Angle Slew Rate Limiter (Maksimal perubahan 300 deg/s untuk respon gesit tanpa hunting)
        MAX_ANGLE_RATE = math.radians(300.0)
        if not hasattr(self, 'prev_theta_ref'):
            self.prev_theta_ref = 0.0
            self.prev_phi_ref = 0.0

        d_theta = np.clip(theta_ref - self.prev_theta_ref, -MAX_ANGLE_RATE * dt_control, MAX_ANGLE_RATE * dt_control)
        theta_ref = self.prev_theta_ref + d_theta
        self.prev_theta_ref = theta_ref

        d_phi = np.clip(phi_ref - self.prev_phi_ref, -MAX_ANGLE_RATE * dt_control, MAX_ANGLE_RATE * dt_control)
        phi_ref = self.prev_phi_ref + d_phi
        self.prev_phi_ref = phi_ref

        # 2. Attitude Safety Recovery Cutoff: jika sudut > 30 deg, paksa level out dan reset integral
        if abs(phi) > math.radians(30.0) or abs(theta) > math.radians(30.0):
            theta_ref = 0.0
            phi_ref = 0.0
            self.pid_x_out.integral = 0.0
            self.pid_y_out.integral = 0.0
            self.prev_theta_ref = 0.0
            self.prev_phi_ref = 0.0

        err_theta = theta_ref - theta
        uy_pid = np.clip(self.pid_x_in.Kp * err_theta - self.pid_x_in.Kd * q_ang, -self.limits['tau_rp_max'], self.limits['tau_rp_max'])
        
        err_phi = phi_ref - phi
        ux_pid = np.clip(self.pid_y_in.Kp * err_phi - self.pid_y_in.Kd * p, -self.limits['tau_rp_max'], self.limits['tau_rp_max'])
        
        # Altitude Outer/Inner Loop — Derivative on Measurement (D-term langsung dari vz)
        err_z = self.filt_z[0] - z
        p_term_z = self.pid_z.Kp * err_z
        if z >= 0.15:
            self.pid_z.integral += err_z * dt_control
            if self.pid_z.i_max < np.inf:
                self.pid_z.integral = float(np.clip(self.pid_z.integral, -self.pid_z.i_max, self.pid_z.i_max))
        i_term_z = self.pid_z.Ki * self.pid_z.integral
        d_term_z = -self.pid_z.Kd * vz
        uz_pid = np.clip(p_term_z + i_term_z + d_term_z, -self.limits['thrust_max'], self.limits['thrust_max'])
        
        # Angle-Thrust Compensation (compensates for vertical force loss due to tilt)
        cos_phi = math.cos(phi)
        cos_theta = math.cos(theta)
        tilt_comp = 1.0 / max(cos_phi * cos_theta, 0.7)
        u_thrust = (uz_pid + (self.m * self.g)) * tilt_comp
        
        # Normalisasi error yaw ke range [-pi, pi] untuk menghindari loncat 2pi
        err_yaw = (yaw_cmd_norm - yaw + np.pi) % (2 * np.pi) - np.pi
        uyaw_pid = np.clip(self.pid_yaw.Kp * err_yaw - self.pid_yaw.Kd * r_ang + self.k_ff_yaw * self.yaw_rate_cmd, -self.limits['tau_y_max'], self.limits['tau_y_max'])
        
        U_cmd = np.array([u_thrust, ux_pid, uy_pid, uyaw_pid])
        
        # Dynamic Torque Desaturation (Thrust Priority > Attitude Torques)
        # Menjamin seluruh motor selalu berada pada rentang aktif 250 - 1050 rad/s tanpa pernah mati (0 rad/s)
        w_sq_cmd = self.M_inv @ U_cmd
        w_floor_sq = 250.0**2
        w_ceil_sq = 1050.0**2
        if np.min(w_sq_cmd) < w_floor_sq or np.max(w_sq_cmd) > w_ceil_sq:
            for scale_y in [0.70, 0.40, 0.20, 0.0]:
                U_cmd_test = np.array([u_thrust, ux_pid, uy_pid, uyaw_pid * scale_y])
                w_sq_test = self.M_inv @ U_cmd_test
                if np.min(w_sq_test) >= w_floor_sq and np.max(w_sq_test) <= w_ceil_sq:
                    w_sq_cmd = w_sq_test
                    break
            else:
                for scale in [0.85, 0.70, 0.50, 0.30, 0.10, 0.0]:
                    U_cmd_scaled = np.array([u_thrust, ux_pid * scale, uy_pid * scale, 0.0])
                    w_sq_test = self.M_inv @ U_cmd_scaled
                    if np.min(w_sq_test) >= w_floor_sq and np.max(w_sq_test) <= w_ceil_sq:
                        w_sq_cmd = w_sq_test
                        break

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
                f"━━━━━━━━━━━━━━━━━━━ [PID-HInf | T={t:.1f}s] ───────────────────\n"
                f"  Posisi  │  Aktual   │  Target   │  Error\n"
                f"  X       │  {x:+7.3f}m │  {self.x_cmd:+7.3f}m │  {self.x_cmd - x:+7.3f}m\n"
                f"  Y       │  {y:+7.3f}m │  {self.y_cmd:+7.3f}m │  {self.y_cmd - y:+7.3f}m\n"
                f"  Z       │  {z:+7.3f}m │  {self.z_cmd:+7.3f}m │  {self.z_cmd - z:+7.3f}m\n"
                f"  Yaw     │  {yaw_deg:+7.2f}° │  {math.degrees(self.yaw_cmd):+7.2f}° │  {math.degrees(self.yaw_cmd) - yaw_deg:+7.2f}°\n"
                f"  RPM → [{int(w_cmd[0])}, {int(w_cmd[1])}, {int(w_cmd[2])}, {int(w_cmd[3])}]\n"
                f"───────────────────────────────────────────────────────────────────"
            )
        if t - self.last_csv_log_time >= 0.05:  # 20 Hz
            self.csv_writer.writerow([t, x, y, z, roll_deg, pitch_deg, yaw_deg,
                                      self.filt_x[0], self.filt_y[0], self.filt_z[0], math.degrees(yaw_cmd_norm),
                                      vx, vy, vz, p, q_ang, r_ang,
                                      uz_pid, ux_pid, uy_pid, uyaw_pid,
                                      w_cmd[0], w_cmd[1], w_cmd[2], w_cmd[3]])
        self.publish_drone_marker(x, y, z, phi, theta, yaw, msg.pose.pose.orientation, vx, vy, vz)

        # Update and publish trajectory path trail
        if not hasattr(self, 'path_msg'):
            self.path_msg = Path()
            self.path_msg.header.frame_id = 'world'
            self.last_path_pub_time = 0.0

        if (current_time - self.last_path_pub_time) >= 0.08:
            self.last_path_pub_time = current_time
            pose_stamped = PoseStamped()
            pose_stamped.header = msg.header
            pose_stamped.header.frame_id = 'world'
            pose_stamped.pose = msg.pose.pose
            self.path_msg.poses.append(pose_stamped)
            if len(self.path_msg.poses) > 2500:
                self.path_msg.poses.pop(0)
            self.path_msg.header.stamp = msg.header.stamp
            self.path_pub.publish(self.path_msg)

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
        _, _, yaw_target = self.euler_from_quaternion(qx, qy, qz, qw)
        self.yaw_cmd = yaw_target

    def target_velocity_callback(self, msg):
        """Terima kecepatan ORCA dari mid-level sebagai velocity feedforward."""
        self.vx_cmd = float(msg.twist.linear.x)
        self.vy_cmd = float(msg.twist.linear.y)
        self.yaw_rate_cmd = float(msg.twist.angular.z)

    def cmd_vel_callback(self, msg):
        """Terima perintah Twist langsung (Body Frame) dari node mapping/planner."""
        vx_b = float(msg.linear.x)
        vy_b = float(msg.linear.y)
        wz_b = float(msg.angular.z)

        # Konversi Body Frame ke World Frame untuk integrasi posisi & feedforward
        yaw_curr = getattr(self, 'last_yaw', 0.0)
        cos_y = math.cos(yaw_curr)
        sin_y = math.sin(yaw_curr)
        vx_w = vx_b * cos_y - vy_b * sin_y
        vy_w = vx_b * sin_y + vy_b * cos_y

        self.vx_cmd = vx_w
        self.vy_cmd = vy_w
        self.yaw_rate_cmd = wz_b

        # Integrasikan kecepatan ke target posisi x_cmd, y_cmd, yaw_cmd hanya jika target_pose belum aktif
        if not self.target_pose_received:
            dt_cmd = 0.05
            self.x_cmd += self.vx_cmd * dt_cmd
            self.y_cmd += self.vy_cmd * dt_cmd
            self.yaw_cmd += self.yaw_rate_cmd * dt_cmd
            self.target_pose_received = True

    def destroy_node(self):
        self.csv_file.close()
        super().destroy_node()

def main(args=None):
    import sys
    rclpy.init(args=args)
    node = PIDHinfNode()
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
