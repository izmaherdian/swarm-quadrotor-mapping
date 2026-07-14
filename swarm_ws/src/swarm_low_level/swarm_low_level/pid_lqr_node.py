import rclpy
from rclpy.node import Node
from nav_msgs.msg import Odometry
import math
import csv
import os

class PIDLQRNode(Node):
    def __init__(self):
        super().__init__('pid_lqr_node')
        
        # Berlangganan data Odometry dari Gazebo Bridge
        self.subscription = self.create_subscription(
            Odometry,
            '/model/iris_1/odometry',
            self.odom_callback,
            10)
            
        self.get_logger().info("PID-LQR Node / Data Logger Started. Menunggu data Odometry...")
        
        # Persiapan file CSV untuk evaluasi
        self.csv_path = os.path.join(os.getcwd(), 'flight_data_log.csv')
        self.csv_file = open(self.csv_path, mode='w', newline='')
        self.csv_writer = csv.writer(self.csv_file)
        self.csv_writer.writerow(['Time_s', 'X', 'Y', 'Z', 'Roll_deg', 'Pitch_deg', 'Yaw_deg'])
        
        self.start_time = None

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
            
        t = current_time - self.start_time
        
        # Posisi
        x = msg.pose.pose.position.x
        y = msg.pose.pose.position.y
        z = msg.pose.pose.position.z
        
        # Orientasi
        qx = msg.pose.pose.orientation.x
        qy = msg.pose.pose.orientation.y
        qz = msg.pose.pose.orientation.z
        qw = msg.pose.pose.orientation.w
        
        roll, pitch, yaw = self.euler_from_quaternion(qx, qy, qz, qw)
        
        roll_deg = math.degrees(roll)
        pitch_deg = math.degrees(pitch)
        yaw_deg = math.degrees(yaw)
        
        # Cetak ke layar tiap ~0.5 detik (kalau data masuk di 50Hz)
        if int(t * 50) % 25 == 0:
            self.get_logger().info(
                f"T={t:.1f}s | Pos: X={x:.2f}, Y={y:.2f}, Z={z:.2f} | Orient: R={roll_deg:.1f}, P={pitch_deg:.1f}, Y={yaw_deg:.1f}"
            )
            
        # Simpan ke CSV
        self.csv_writer.writerow([t, x, y, z, roll_deg, pitch_deg, yaw_deg])

    def destroy_node(self):
        self.csv_file.close()
        self.get_logger().info(f"Log CSV tersimpan di {self.csv_path}")
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
