import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Point, PointStamped
from swarm_msgs.msg import Heartbeat
import numpy as np

# Sutherland-Hodgman / Bisector Line Clipper untuk partisi Voronoi 2D
def clip_polygon_by_bisector(polygon, pi, pj):
    """
    Memotong polygon (list koordinat 2D) dengan garis pembagi tegak lurus (perpendicular bisector)
    antara titik pi (drone saat ini) dan pj (drone tetangga).
    Menyimpan bagian polygon yang berisi titik pi.
    """
    clipped = []
    if len(polygon) == 0:
        return clipped
        
    midpoint = (pi + pj) / 2.0
    normal = pj - pi # Vektor normal menunjuk ke pj (arah luar)
    
    # Fungsi penentu sisi: negatif jika berada di sisi pi (dalam), positif jika di pj (luar)
    def is_inside(p):
        return np.dot(p - midpoint, normal) <= 0.0

    # Cari titik potong segmen (s1 -> s2) dengan garis pembagi
    def line_intersection(s1, s2):
        # Persamaan garis bisector: dot(P - midpoint, normal) = 0
        # P(t) = s1 + t * (s2 - s1)
        # dot(s1 + t*(s2-s1) - midpoint, normal) = 0
        # t = dot(midpoint - s1, normal) / dot(s2 - s1, normal)
        ds = s2 - s1
        denom = np.dot(ds, normal)
        if abs(denom) < 1e-9:
            return s1
        t = np.dot(midpoint - s1, normal) / denom
        return s1 + t * ds

    s1 = polygon[-1]
    for s2 in polygon:
        if is_inside(s2):
            if not is_inside(s1):
                clipped.append(line_intersection(s1, s2))
            clipped.append(s2)
        elif is_inside(s1):
            clipped.append(line_intersection(s1, s2))
        s1 = s2
        
    return clipped

def calculate_polygon_centroid(polygon):
    """
    Menghitung koordinat centroid (pusat massa) dari polygon 2D.
    """
    if len(polygon) < 3:
        # Jika bukan area tertutup (kurang dari 3 titik), gunakan rata-rata koordinat
        return np.mean(polygon, axis=0) if len(polygon) > 0 else np.array([0.0, 0.0])
        
    # Rumus Centroid Polygon 2D standar (Gauss Shoelace)
    area = 0.0
    cx = 0.0
    cy = 0.0
    
    n = len(polygon)
    for i in range(n):
        x0, y0 = polygon[i]
        x1, y1 = polygon[(i + 1) % n]
        
        factor = (x0 * y1 - x1 * y0)
        area += factor
        cx += (x0 + x1) * factor
        cy += (y0 + y1) * factor
        
    area = 0.5 * area
    if abs(area) < 1e-9:
        return np.mean(polygon, axis=0)
        
    cx = cx / (6.0 * area)
    cy = cy / (6.0 * area)
    return np.array([cx, cy])


class VoronoiNode(Node):
    def __init__(self):
        super().__init__('voronoi_node')
        
        # 1. Deklarasi Parameter Area & ID
        self.declare_parameter('drone_id', 1)
        self.declare_parameter('num_drones', 3)
        self.declare_parameter('x_min', 0.0)
        self.declare_parameter('x_max', 10.0)
        self.declare_parameter('y_min', -5.0)
        self.declare_parameter('y_max', 5.0)
        
        self.drone_id = self.get_parameter('drone_id').value
        self.num_drones = self.get_parameter('num_drones').value
        self.x_min = self.get_parameter('x_min').value
        self.x_max = self.get_parameter('x_max').value
        self.y_min = self.get_parameter('y_min').value
        self.y_max = self.get_parameter('y_max').value
        
        self.get_logger().info(
            f"Node Voronoi aktif untuk Drone_{self.drone_id}. "
            f"Batas area X: [{self.x_min}, {self.x_max}], Y: [{self.y_min}, {self.y_max}]"
        )
        
        # Bounding box sebagai polygon awal (counter-clockwise)
        self.initial_bbox = np.array([
            [self.x_min, self.y_min],
            [self.x_max, self.y_min],
            [self.x_max, self.y_max],
            [self.x_min, self.y_max]
        ])
        
        # 2. State Posisi Drone (Lokal & Tetangga)
        self.drone_positions = {} # {id: np.array([x, y])}
        self.drone_positions[self.drone_id] = np.array([0.0, 0.0])
        
        # 3. Subscriber Heartbeat P2P
        # Dapatkan info posisi dan keaktifan tetangga
        self.heartbeat_subs = {}
        for i in range(1, self.num_drones + 1):
            self.create_heartbeat_subscriber(i)

        # 4. Publisher Centroid Voronoi
        self.centroid_pub = self.create_publisher(
            PointStamped,
            f'/iris_{self.drone_id}/voronoi_centroid',
            10
        )
        
        # 5. Timer Perhitungan Voronoi: 10Hz (0.1s)
        self.calc_timer = self.create_timer(0.1, self.compute_voronoi_cell)

    def create_heartbeat_subscriber(self, i):
        topic_name = f'/iris_{i}/heartbeat'
        
        def cb(msg):
            # Callback untuk memantau data posisi & keaktifan drone lain
            if msg.is_active:
                self.drone_positions[msg.drone_id] = np.array([msg.position.x, msg.position.y])
            elif msg.drone_id in self.drone_positions and msg.drone_id != self.drone_id:
                # Jika status tidak aktif, hapus dari list perhitungan Voronoi (FT-CC)
                del self.drone_positions[msg.drone_id]
                self.get_logger().warn(f"[FT-CC] Mengecualikan Drone_{msg.drone_id} dari partisi Voronoi!")
                
        self.heartbeat_subs[i] = self.create_subscription(
            Heartbeat,
            topic_name,
            cb,
            10
        )

    def compute_voronoi_cell(self):
        # 1. Pastikan posisi drone sendiri sudah tersedia
        pi = self.drone_positions.get(self.drone_id)
        if pi is None:
            return
            
        # 2. Mulai dengan polygon bounding box utuh
        cell_polygon = list(self.initial_bbox)
        
        # 3. Potong polygon dengan pembagi tegak lurus (bisector) terhadap tetangga yang AKTIF
        for j, pj in self.drone_positions.items():
            if j == self.drone_id:
                continue
            cell_polygon = clip_polygon_by_bisector(cell_polygon, pi, pj)
            
        # 4. Hitung Centroid dari Voronoi cell yang dihasilkan
        if len(cell_polygon) > 0:
            centroid = calculate_polygon_centroid(cell_polygon)
            
            # 5. Publikasikan koordinat Centroid ke topik
            msg = PointStamped()
            msg.header.stamp = self.get_clock().now().to_msg()
            msg.header.frame_id = 'world'
            msg.point.x = float(centroid[0])
            msg.point.y = float(centroid[1])
            msg.point.z = 2.0 # Ketinggian terbang target (Z=2m)
            self.centroid_pub.publish(msg)
            
            # Log debug sesekali
            self._debug_count = getattr(self, '_debug_count', 0) + 1
            if self._debug_count % 30 == 0:
                self.get_logger().info(
                    f"Drone_{self.drone_id} Pos:({pi[0]:.2f},{pi[1]:.2f}) "
                    f"→ Centroid Voronoi:({centroid[0]:.2f},{centroid[1]:.2f}), "
                    f"Tetangga Aktif: {list(self.drone_positions.keys())}"
                )

def main(args=None):
    rclpy.init(args=args)
    node = VoronoiNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
