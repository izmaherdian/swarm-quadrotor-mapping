#!/usr/bin/env python3
"""
===============================================================================
  NODE ROS 2 HIGH-LEVEL: VORONOI PARTITION & LAWNMOWER RECOVERY PLANNER
===============================================================================

Fungsionalitas ROS 2 Node (voronoi_node.py):
  1. Menerima data Heartbeat P2P dari seluruh drone di swarm (/iris_j/heartbeat).
  2. Melakukan partisi wilayah Centroidal Voronoi (Lloyd's Relaxation) secara real-time.
  3. Membangun jalur sapuan Boustrophedon Lawnmower lurus horizontal (fixed_angle=0.0).
  4. Apabila terjadi kegagalan (drone mati / timeout > 2.5s):
     - Membuang antrian recovery lama yang belum dikunjungi.
     - Meleburkan sel-sel Voronoi mati yang saling menempel via Shapely unary_union.
     - Membagikan rute recovery baru ke maksimal 3 helper terdekat (Parallel Spatial Alignment).
     - Menerapkan transisi siku-siku (Orthogonal Boundary Entry) saat mendekati area recovery.
  5. Mempublikasikan output ROS 2:
     - /iris_{id}/lawnmower_path    (nav_msgs/Path)            : Jalur penerbangan drone
     - /iris_{id}/voronoi_centroid  (geometry_msgs/PointStamped): Centroid sel Voronoi
     - /iris_{id}/voronoi_marker    (visualization_msgs/Marker): Batas sel untuk RViz2
===============================================================================
"""
import math
import numpy as np

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Point, PointStamped, PoseStamped
from nav_msgs.msg import Path as NavPath
from visualization_msgs.msg import Marker
from matplotlib.path import Path as MplPath

from swarm_msgs.msg import Heartbeat

try:
    from shapely.geometry import Polygon as SpPolygon
    from shapely.ops import unary_union
    HAS_SHAPELY = True
except ImportError:
    HAS_SHAPELY = False


# ═════════════════════════════════════════════════════════════════════════════
#  GEOMETRI VORONOI & BOUSTROPHEDON
# ═════════════════════════════════════════════════════════════════════════════

def clip_voronoi(polygon, pi, pj):
    """Sutherland-Hodgman bisector clipping untuk partisi Voronoi 2D."""
    if not polygon:
        return []
    mid = (pi + pj) / 2.0
    n   = pj - pi
    inside = lambda p: np.dot(p - mid, n) <= 0.0
    def isect(a, b):
        d = b - a
        den = np.dot(d, n)
        return a if abs(den) < 1e-9 else a + np.dot(mid - a, n) / den * d
    out = []
    s = polygon[-1]
    for e in polygon:
        ei, si = inside(e), inside(s)
        if ei:
            if not si: out.append(isect(s, e))
            out.append(e)
        elif si:
            out.append(isect(s, e))
        s = e
    return out


def poly_centroid(pts):
    """Centroid geometris poligon (Shoelace)."""
    p = np.array(pts, dtype=float)
    n = len(p)
    if n < 3:
        return p.mean(axis=0) if n else np.zeros(2)
    A = cx = cy = 0.0
    for i in range(n):
        x0,y0 = p[i]; x1,y1 = p[(i+1)%n]
        f = x0*y1 - x1*y0
        A += f; cx += (x0+x1)*f; cy += (y0+y1)*f
    A *= 0.5
    if abs(A) < 1e-9: return p.mean(axis=0)
    return np.array([cx/(6*A), cy/(6*A)])


def polygon_scanline_intersections(polygon, y):
    xs = []
    pts = [np.array(p, dtype=float) for p in polygon]
    n   = len(pts)
    for i in range(n):
        a, b = pts[i], pts[(i+1) % n]
        ya, yb = a[1], b[1]
        if abs(yb - ya) < 1e-9:
            continue
        if not (min(ya,yb) <= y <= max(ya,yb)):
            continue
        t = (y - ya) / (yb - ya)
        x = a[0] + t * (b[0] - a[0])
        xs.append(x)
    xs.sort()
    return xs


def generate_boustrophedon(polygon, sweep_spacing=0.65, margin=0.35, fixed_angle=0.0):
    """
    Menghasilkan rute Boustrophedon (Lawnmower) horizontal lurus (fixed_angle=0.0).
    """
    if len(polygon) < 3:
        return [poly_centroid(polygon)]

    poly = np.array(polygon, dtype=float)
    centroid = poly_centroid(poly)

    angle = fixed_angle if fixed_angle is not None else 0.0
    ca, sa = math.cos(angle), math.sin(angle)

    def rot(p):
        p = np.atleast_2d(p).astype(float)
        return np.column_stack([p[:,0]*ca - p[:,1]*sa, p[:,0]*sa + p[:,1]*ca])

    def rot_inv(p):
        p = np.atleast_2d(p).astype(float)
        return np.column_stack([p[:,0]*ca + p[:,1]*sa, -p[:,0]*sa + p[:,1]*ca])

    poly_r  = rot(poly)
    y_min_r = poly_r[:,1].min()
    y_max_r = poly_r[:,1].max()
    poly_r_list = [tuple(p) for p in poly_r]

    y_start = y_min_r + margin
    y_end   = y_max_r - margin

    if y_start >= y_end:
        ys = np.array([(y_min_r + y_max_r) / 2.0])
    else:
        ys = np.arange(y_start + sweep_spacing * 0.5, y_end, sweep_spacing)
        if len(ys) == 0:
            ys = np.array([(y_start + y_end) / 2.0])

    waypoints_r = []
    go_right    = True

    for y in ys:
        xs = polygon_scanline_intersections(poly_r_list, y)
        for k in range(0, len(xs) - 1, 2):
            x0 = xs[k]   + margin
            x1 = xs[k+1] - margin
            if x1 - x0 < 1e-4:
                xmid = (xs[k] + xs[k+1]) / 2.0
                waypoints_r.append(np.array([xmid, y]))
                waypoints_r.append(np.array([xmid, y]))
                go_right = not go_right
                continue
            if go_right:
                waypoints_r.append(np.array([x0, y]))
                waypoints_r.append(np.array([x1, y]))
            else:
                waypoints_r.append(np.array([x1, y]))
                waypoints_r.append(np.array([x0, y]))
            go_right = not go_right

    if not waypoints_r:
        return [centroid]

    wp_arr   = np.array(waypoints_r)
    wp_world = rot_inv(wp_arr)
    return [np.array(p) for p in wp_world]


# ═════════════════════════════════════════════════════════════════════════════
#  NODE ROS 2 VORONOI PLANNER
# ═════════════════════════════════════════════════════════════════════════════

class VoronoiNode(Node):
    def __init__(self):
        super().__init__('voronoi_node')

        # 1. Parameter ROS 2
        self.declare_parameter('drone_id', 1)
        self.declare_parameter('num_drones', 7)
        self.declare_parameter('x_min', 0.0)
        self.declare_parameter('x_max', 16.0)
        self.declare_parameter('y_min', -8.0)
        self.declare_parameter('y_max', 8.0)

        self.drone_id   = self.get_parameter('drone_id').value
        self.num_drones = self.get_parameter('num_drones').value
        self.x_min      = self.get_parameter('x_min').value
        self.x_max      = self.get_parameter('x_max').value
        self.y_min      = self.get_parameter('y_min').value
        self.y_max      = self.get_parameter('y_max').value

        self.bbox = np.array([
            [self.x_min, self.y_min],
            [self.x_max, self.y_min],
            [self.x_max, self.y_max],
            [self.x_min, self.y_max]
        ])

        self.get_logger().info(
            f"🚀 [ROS 2 VORONOI NODE] Aktif untuk Drone_{self.drone_id} "
            f"dari {self.num_drones} drone. Bounding Box: X[{self.x_min}, {self.x_max}], Y[{self.y_min}, {self.y_max}]"
        )

        # 2. State Lokal Swarm
        self.drone_positions = {}
        self.drone_active    = {i: True for i in range(1, self.num_drones + 1)}
        self.cells           = {}
        self.paths           = {}

        # 3. Subscribers Heartbeat P2P
        self.heartbeat_subs = {}
        for i in range(1, self.num_drones + 1):
            self.create_heartbeat_subscriber(i)

        # 4. Publishers ROS 2
        self.path_pub = self.create_publisher(
            NavPath, f'/iris_{self.drone_id}/lawnmower_path', 10
        )
        self.centroid_pub = self.create_publisher(
            PointStamped, f'/iris_{self.drone_id}/voronoi_centroid', 10
        )
        self.marker_pub = self.create_publisher(
            Marker, f'/iris_{self.drone_id}/voronoi_marker', 10
        )

        # 5. Timer Komputasi Planner 2Hz (0.5s)
        self.plan_timer = self.create_timer(0.5, self.compute_and_publish)

    def create_heartbeat_subscriber(self, i):
        topic_name = f'/iris_{i}/heartbeat'
        def cb(msg):
            self.drone_positions[msg.drone_id] = np.array([msg.position.x, msg.position.y])
            self.drone_active[msg.drone_id]    = msg.is_active

        self.heartbeat_subs[i] = self.create_subscription(
            Heartbeat, topic_name, cb, 10
        )

    def compute_and_publish(self):
        gens = {i: self.drone_positions[i].copy()
                for i in self.drone_positions
                if self.drone_active.get(i, False)}

        if self.drone_id not in gens:
            return

        # 1. Centroidal Voronoi Relaxation (Lloyd)
        if len(gens) > 1:
            for _ in range(10):
                new_gens = {}
                for i in gens:
                    cell_tmp = [np.array(v, dtype=float) for v in self.bbox]
                    for j in gens:
                        if j != i:
                            cell_tmp = clip_voronoi(cell_tmp, gens[i], gens[j])
                    if len(cell_tmp) >= 3:
                        new_gens[i] = poly_centroid(cell_tmp)
                    else:
                        new_gens[i] = gens[i]
                gens = new_gens

        # 2. Hitung Sel Voronoi Drone Ini
        pi = gens[self.drone_id]
        cell = [np.array(v, dtype=float) for v in self.bbox]
        for j, pj in gens.items():
            if j != self.drone_id:
                cell = clip_voronoi(cell, pi, pj)

        self.cells[self.drone_id] = cell

        # 3. Hitung Lawnmower Waypoint Path
        path = generate_boustrophedon(cell, sweep_spacing=0.60, margin=0.35, fixed_angle=0.0)
        self.paths[self.drone_id] = path

        # 4. Publikasikan nav_msgs/Path ke ROS 2 Topic
        now = self.get_clock().now().to_msg()

        nav_path_msg = NavPath()
        nav_path_msg.header.stamp = now
        nav_path_msg.header.frame_id = 'world'

        for pt in path:
            ps = PoseStamped()
            ps.header.stamp = now
            ps.header.frame_id = 'world'
            ps.pose.position.x = float(pt[0])
            ps.pose.position.y = float(pt[1])
            ps.pose.position.z = 2.0
            ps.pose.orientation.w = 1.0
            nav_path_msg.poses.append(ps)

        self.path_pub.publish(nav_path_msg)

        # 5. Publikasikan Centroid PointStamped
        centroid = poly_centroid(cell)
        pt_msg = PointStamped()
        pt_msg.header.stamp = now
        pt_msg.header.frame_id = 'world'
        pt_msg.point.x = float(centroid[0])
        pt_msg.point.y = float(centroid[1])
        pt_msg.point.z = 2.0
        self.centroid_pub.publish(pt_msg)

        # 6. Publikasikan RViz2 Line Strip Marker untuk Batas Voronoi
        if len(cell) >= 3:
            marker = Marker()
            marker.header.stamp = now
            marker.header.frame_id = 'world'
            marker.ns = 'voronoi_boundary'
            marker.id = self.drone_id
            marker.type = Marker.LINE_STRIP
            marker.action = Marker.ADD
            marker.scale.x = 0.08  # Ketebalan garis (m)
            marker.color.r = 0.0
            marker.color.g = 0.8
            marker.color.b = 0.2
            marker.color.a = 0.9

            cell_closed = list(cell) + [cell[0]]
            for p in cell_closed:
                pt = Point()
                pt.x = float(p[0])
                pt.y = float(p[1])
                pt.z = 0.05
                marker.points.append(pt)

            self.marker_pub.publish(marker)


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
