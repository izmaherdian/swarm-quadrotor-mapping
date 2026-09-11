#!/usr/bin/env python3
"""
Generator Otomatis Berkas World Gazebo Harmonic untuk Swarm Quadrotor Mapping.
Palet Warna Profesional & Elegan (Industrial Matte / Architectural Theme).

Menghasilkan tampilan visual 3D yang 100% presisi mengikuti bentuk wilayah:
- `rect` (Persegi 28x28m)
- `l_shape` (L-Shape Non-Konveks)
- `u_shape` (U-Shape Berkantong Cekung Non-Konveks)
- `plus` (Salib Simetris 4 Lengan Non-Konveks)

Mendukung seluruh skema:
1. empty_<region>.world            (Skema 1 & 2)
2. obstacles_<region>.world        (Skema 3)
3. obstacles_dynamic_<region>.world (Skema 4 & 5)
"""

import math
import os
import pathlib
import sys
import shutil

WS = pathlib.Path(__file__).resolve().parent.parent
SRC = WS / 'src' / 'swarm_high_level'
if str(SRC) not in sys.path:
    sys.path.insert(0, str(SRC))

from swarm_high_level.world.region import REGION_PRESETS
from swarm_high_level.world.obstacles import (
    OBSTACLES_BY_REGION, OBSTACLE_HEIGHT, OBSTACLE_RADIUS
)

# Palet Rintangan Statis Profesional (Muted Industrial Matte Colors)
PROFESSIONAL_STATIC_COLORS = [
    (0.40, 0.45, 0.50),  # Steel Slate
    (0.48, 0.42, 0.38),  # Warm Bronze Slate
    (0.35, 0.45, 0.45),  # Muted Sage/Teal
    (0.45, 0.40, 0.48),  # Muted Indigo
    (0.50, 0.45, 0.38),  # Dark Ochre
    (0.38, 0.46, 0.40),  # Olive Slate
    (0.42, 0.44, 0.48),  # Iron Grey
    (0.48, 0.38, 0.40),  # Muted Burgundy
    (0.38, 0.42, 0.46),  # Deep Steel
]

WORLDS_SRC = WS / 'src' / 'swarm_sim' / 'worlds'

# Dekomposisi kotak lantai visual (Covering Boxes) untuk setiap wilayah
REGION_FLOOR_BOXES = {
    'rect': [
        # (center_x, center_y, size_x, size_y)
        (0.0, 0.0, 28.0, 28.0),
    ],
    'l_shape': [
        # Paruh bawah (X: -14..14, Y: -14..0)
        (0.0, -7.0, 28.0, 14.0),
        # Paruh kiri atas (X: -14..0, Y: 0..14)
        (-7.0, 7.0, 14.0, 14.0),
    ],
    'u_shape': [
        # Bagian dasar bawah (X: -14..14, Y: -14..-2)
        (0.0, -8.0, 28.0, 12.0),
        # Lengan kiri (X: -14..-5, Y: -2..14)
        (-9.5, 6.0, 9.0, 16.0),
        # Lengan kanan (X: 5..14, Y: -2..14)
        (9.5, 6.0, 9.0, 16.0),
    ],
    'plus': [
        # Kotak pusat (X: -5..5, Y: -5..5)
        (0.0, 0.0, 10.0, 10.0),
        # Lengan utara (X: -5..5, Y: 5..14)
        (0.0, 9.5, 10.0, 9.0),
        # Lengan selatan (X: -5..5, Y: -14..-5)
        (0.0, -9.5, 10.0, 9.0),
        # Lengan timur (X: 5..14, Y: -5..5)
        (9.5, 0.0, 9.0, 10.0),
        # Lengan barat (X: -14..-5, Y: -5..5)
        (-9.5, 0.0, 9.0, 10.0),
    ]
}

HEADER_TEMPLATE = """<?xml version="1.0" ?>
<sdf version="1.8">
  <world name="swarm_world">
    <!-- SPEEDUP SIMULATION: max_step_size=0.001 (1000Hz stable physics) & real_time_factor=1.0 -->
    <physics name="fast_physics" type="ode">
      <max_step_size>0.001</max_step_size>
      <real_time_factor>1.0</real_time_factor>
    </physics>

    <!-- SCENE CONFIG: Muted Architectural Lighting & Background -->
    <scene>
      <ambient>0.62 0.62 0.65 1.0</ambient>
      <background>0.78 0.80 0.83 1.0</background>
      <shadows>true</shadows>
      <grid>false</grid>
    </scene>

    <!-- PLUGIN INTI GAZEBO HARMONIC -->
    <plugin
      filename="gz-sim-physics-system"
      name="gz::sim::systems::Physics">
    </plugin>
    <plugin
      filename="gz-sim-user-commands-system"
      name="gz::sim::systems::UserCommands">
    </plugin>
    <plugin
      filename="gz-sim-scene-broadcaster-system"
      name="gz::sim::systems::SceneBroadcaster">
    </plugin>
    <plugin
      filename="gz-sim-sensors-system"
      name="gz::sim::systems::Sensors">
      <render_engine>ogre2</render_engine>
    </plugin>
    <plugin
      filename="gz-sim-wind-effects-system"
      name="gz::sim::systems::WindEffects">
      <force_approximation_scaling_factor>0.25</force_approximation_scaling_factor>
    </plugin>

    <!-- MATAHARI (SUN) - Soft Balanced Illumination -->
    <light type="directional" name="sun">
      <cast_shadows>true</cast_shadows>
      <pose>0 0 30 0 0 0</pose>
      <diffuse>0.85 0.85 0.87 1</diffuse>
      <specular>0.2 0.2 0.2 1</specular>
      <attenuation>
        <range>1000</range>
        <constant>0.9</constant>
        <linear>0.01</linear>
        <quadratic>0.001</quadratic>
      </attenuation>
      <direction>-0.4 0.2 -0.9</direction>
    </light>

    <!-- LANTAI LUAR (NEUTRAL MATTE CONCRETE EXTERIOR) -->
    <model name="ground_plane">
      <static>true</static>
      <link name="link">
        <collision name="collision">
          <geometry>
            <plane>
              <normal>0 0 1</normal>
              <size>100 100</size>
            </plane>
          </geometry>
        </collision>
        <visual name="visual">
          <geometry>
            <plane>
              <normal>0 0 1</normal>
              <size>100 100</size>
            </plane>
          </geometry>
          <material>
            <ambient>0.70 0.72 0.74 1.0</ambient>
            <diffuse>0.70 0.72 0.74 1.0</diffuse>
            <specular>0.1 0.1 0.1 1.0</specular>
          </material>
        </visual>
      </link>
    </model>

    <!-- GUI TOP-VIEW CAMERA PRESET -->
    <gui fullscreen="0">
      <camera name="user_camera">
        <pose>0.0 0.0 38.0 0.0 1.5708 1.5708</pose>
        <view_controller>orbit</view_controller>
      </camera>
    </gui>

    <!-- STAGING PAD AREA (LANDASAN AWAL 7 DRONE DI Y = -18m) -->
    <model name="staging_pad">
      <static>true</static>
      <link name="pad_link">
        <visual name="pad_v">
          <pose>0.0 -17.25 0.003 0 0 0</pose>
          <geometry><box><size>10.0 3.5 0.005</size></box></geometry>
          <material>
            <ambient>0.26 0.28 0.32 1.0</ambient>
            <diffuse>0.26 0.28 0.32 1.0</diffuse>
          </material>
        </visual>
        <visual name="pad_border">
          <pose>0.0 -17.25 0.005 0 0 0</pose>
          <geometry><box><size>10.1 3.6 0.003</size></box></geometry>
          <material>
            <ambient>0.38 0.44 0.50 1.0</ambient>
            <diffuse>0.38 0.44 0.50 1.0</diffuse>
          </material>
        </visual>
      </link>
    </model>
"""

STATIC_OBS_TEMPLATE = """\
    <model name="static_obs_{oid}">
      <static>true</static>
      <pose>{x:.4f} {y:.4f} {z:.4f} 0 0 0</pose>
      <link name="link">
        <collision name="collision">
          <geometry><cylinder><radius>{r:.4f}</radius><length>{h:.4f}</length></cylinder></geometry>
        </collision>
        <visual name="visual">
          <geometry><cylinder><radius>{r:.4f}</radius><length>{h:.4f}</length></cylinder></geometry>
          <material>
            <ambient>{cr:.2f} {cg:.2f} {cb:.2f} 1.0</ambient>
            <diffuse>{cr:.2f} {cg:.2f} {cb:.2f} 1.0</diffuse>
            <specular>0.3 0.3 0.3 1.0</specular>
          </material>
        </visual>
      </link>
    </model>
"""

DYNAMIC_OBS_MODELS = """\
    <!-- ===================================================================== -->
    <!-- 2 RINTANGAN DINAMIS (DYNAMIC MOVING OBSTACLES) POLA SILANG "X"       -->
    <!-- Warna Muted Industrial Hazard: Dark Crimson & Warm Amber              -->
    <!-- ===================================================================== -->
    <model name="dynamic_obs_1">
      <static>false</static>
      <pose>-10.0 10.0 2.05 0 0 0</pose>
      <link name="link">
        <gravity>false</gravity>
        <kinematic>true</kinematic>
        <inertial>
          <mass>10.0</mass>
          <inertia>
            <ixx>1.0</ixx><ixy>0</ixy><ixz>0</ixz>
            <iyy>1.0</iyy><iyz>0</iyz><izz>1.0</izz>
          </inertia>
        </inertial>
        <collision name="collision">
          <geometry><cylinder><radius>0.45</radius><length>3.6</length></cylinder></geometry>
        </collision>
        <visual name="visual">
          <geometry><cylinder><radius>0.45</radius><length>3.6</length></cylinder></geometry>
          <material>
            <ambient>0.65 0.22 0.22 1.0</ambient>
            <diffuse>0.65 0.22 0.22 1.0</diffuse>
            <specular>0.4 0.4 0.4 1.0</specular>
          </material>
        </visual>
      </link>
      <plugin
        filename="gz-sim-velocity-control-system"
        name="gz::sim::systems::VelocityControl">
      </plugin>
      <plugin
        filename="gz-sim-odometry-publisher-system"
        name="gz::sim::systems::OdometryPublisher">
        <odom_frame>world</odom_frame>
        <robot_base_frame>link</robot_base_frame>
        <odom_publish_frequency>50</odom_publish_frequency>
        <odom_topic>/model/dynamic_obs_1/odometry</odom_topic>
      </plugin>
    </model>

    <model name="dynamic_obs_2">
      <static>false</static>
      <pose>10.0 10.0 2.05 0 0 0</pose>
      <link name="link">
        <gravity>false</gravity>
        <kinematic>true</kinematic>
        <inertial>
          <mass>10.0</mass>
          <inertia>
            <ixx>1.0</ixx><ixy>0</ixy><ixz>0</ixz>
            <iyy>1.0</iyy><iyz>0</iyz><izz>1.0</izz>
          </inertia>
        </inertial>
        <collision name="collision">
          <geometry><cylinder><radius>0.45</radius><length>3.6</length></cylinder></geometry>
        </collision>
        <visual name="visual">
          <geometry><cylinder><radius>0.45</radius><length>3.6</length></cylinder></geometry>
          <material>
            <ambient>0.70 0.42 0.18 1.0</ambient>
            <diffuse>0.70 0.42 0.18 1.0</diffuse>
            <specular>0.4 0.4 0.4 1.0</specular>
          </material>
        </visual>
      </link>
      <plugin
        filename="gz-sim-velocity-control-system"
        name="gz::sim::systems::VelocityControl">
      </plugin>
      <plugin
        filename="gz-sim-odometry-publisher-system"
        name="gz::sim::systems::OdometryPublisher">
        <odom_frame>world</odom_frame>
        <robot_base_frame>link</robot_base_frame>
        <odom_publish_frequency>50</odom_publish_frequency>
        <odom_topic>/model/dynamic_obs_2/odometry</odom_topic>
      </plugin>
    </model>
"""

FOOTER = """\
  </world>
</sdf>
"""

def generate_visual_region_models(region: str) -> str:
    """Menghasilkan model SDF Lantai Khusus & Border Garis Keliling Profesional."""
    poly_verts = REGION_PRESETS[region]
    boxes = REGION_FLOOR_BOXES[region]
    
    xml = [
        f'    <!-- ===================================================================== -->',
        f'    <!-- VISUAL LANTAI ARENA & BORDER WILAYAH {region.upper()}                           -->',
        f'    <!-- ===================================================================== -->',
        f'    <model name="visual_region_floor_{region}">',
        f'      <static>true</static>',
        f'      <link name="floor_link">',
    ]
    
    # 1. Kotak-kotak Lantai Aktif (Muted Charcoal Slate)
    for idx, (cx, cy, sx, sy) in enumerate(boxes):
        xml.extend([
            f'        <visual name="floor_box_{idx}">',
            f'          <pose>{cx:.4f} {cy:.4f} 0.003 0 0 0</pose>',
            f'          <geometry><box><size>{sx:.4f} {sy:.4f} 0.005</size></box></geometry>',
            f'          <material>',
            f'            <ambient>0.26 0.28 0.32 1.0</ambient>',
            f'            <diffuse>0.26 0.28 0.32 1.0</diffuse>',
            f'            <specular>0.15 0.15 0.15 1.0</specular>',
            f'          </material>',
            f'        </visual>',
        ])
        
    # 2. Grid Garis Internal 1m x 1m di Dalam Kotak-Kotak Lantai (Subtle Slate Lines)
    grid_line_idx = 0
    for (cx, cy, sx, sy) in boxes:
        x_start = cx - sx / 2.0
        x_end = cx + sx / 2.0
        y_start = cy - sy / 2.0
        y_end = cy + sy / 2.0
        
        # Garis Vertikal (sejajar sumbu Y)
        curr_x = math.ceil(x_start)
        while curr_x <= math.floor(x_end):
            xml.extend([
                f'        <visual name="grid_v_{grid_line_idx}">',
                f'          <pose>{curr_x:.2f} {cy:.4f} 0.005 0 0 0</pose>',
                f'          <geometry><box><size>0.025 {sy:.4f} 0.003</size></box></geometry>',
                f'          <material>',
                f'            <ambient>0.33 0.35 0.39 1.0</ambient>',
                f'            <diffuse>0.33 0.35 0.39 1.0</diffuse>',
                f'          </material>',
                f'        </visual>',
            ])
            grid_line_idx += 1
            curr_x += 1.0

        # Garis Horizontal (sejajar sumbu X)
        curr_y = math.ceil(y_start)
        while curr_y <= math.floor(y_end):
            xml.extend([
                f'        <visual name="grid_h_{grid_line_idx}">',
                f'          <pose>{cx:.4f} {curr_y:.2f} 0.005 0 0 0</pose>',
                f'          <geometry><box><size>{sx:.4f} 0.025 0.003</size></box></geometry>',
                f'          <material>',
                f'            <ambient>0.33 0.35 0.39 1.0</ambient>',
                f'            <diffuse>0.33 0.35 0.39 1.0</diffuse>',
                f'          </material>',
                f'        </visual>',
            ])
            grid_line_idx += 1
            curr_y += 1.0

    xml.extend([
        f'      </link>',
        f'    </model>',
        f'',
        f'    <!-- BORDER PEMBATAS KELILING (MUTED STEEL BLUE / SLATE CURB) -->',
        f'    <model name="region_perimeter_border_{region}">',
        f'      <static>true</static>',
        f'      <link name="border_link">',
    ])
    
    # 3. Garis Border Keliling Mengelilingi Poligon Wilayah (Classy Steel Slate Curb)
    n_pts = len(poly_verts)
    for i in range(n_pts):
        p1 = poly_verts[i]
        p2 = poly_verts[(i + 1) % n_pts]
        dx = p2[0] - p1[0]
        dy = p2[1] - p1[1]
        length = math.hypot(dx, dy)
        mid_x = (p1[0] + p2[0]) / 2.0
        mid_y = (p1[1] + p2[1]) / 2.0
        yaw = math.atan2(dy, dx)
        
        xml.extend([
            f'        <visual name="border_edge_{i}">',
            f'          <pose>{mid_x:.4f} {mid_y:.4f} 0.015 0 0 {yaw:.4f}</pose>',
            f'          <geometry><box><size>{length:.4f} 0.14 0.025</size></box></geometry>',
            f'          <material>',
            f'            <ambient>0.20 0.36 0.48 1.0</ambient>',
            f'            <diffuse>0.20 0.36 0.48 1.0</diffuse>',
            f'            <specular>0.25 0.25 0.25 1.0</specular>',
            f'          </material>',
            f'        </visual>',
        ])

    xml.extend([
        f'      </link>',
        f'    </model>',
        f'',
    ])
    return '\n'.join(xml)

def generate_static_obstacles(region: str) -> str:
    """Menghasilkan SDF rintangan statis 9 silinder dengan palet warna muted profesional."""
    table = OBSTACLES_BY_REGION.get(region, OBSTACLES_BY_REGION['rect'])
    body = [
        '    <!-- ===================================================================== -->',
        f'    <!-- {len(table)} RINTANGAN STATIS (CYLINDER) — WILAYAH {region.upper()}',
        '    <!-- ===================================================================== -->',
    ]
    for i, (oid, ox, oy) in enumerate(table):
        cr, cg, cb = PROFESSIONAL_STATIC_COLORS[i % len(PROFESSIONAL_STATIC_COLORS)]
        body.append(STATIC_OBS_TEMPLATE.format(
            oid=oid, x=ox, y=oy, z=OBSTACLE_HEIGHT / 2.0,
            r=OBSTACLE_RADIUS, h=OBSTACLE_HEIGHT, cr=cr, cg=cg, cb=cb
        ))
    return '\n'.join(body)

def main():
    os.makedirs(WORLDS_SRC, exist_ok=True)
    
    regions = ['rect', 'l_shape', 'u_shape', 'plus']
    
    for reg in regions:
        vis_models = generate_visual_region_models(reg)
        static_obs = generate_static_obstacles(reg)
        
        # 1. empty_<region>.world (Skema 1 & 2)
        empty_text = HEADER_TEMPLATE + vis_models + FOOTER
        f_empty = WORLDS_SRC / f'empty_{reg}.world'
        f_empty.write_text(empty_text)
        print(f"✅ Generated: {f_empty.name}")
        
        # 2. obstacles_<region>.world (Skema 3 - Statis Saja)
        obs_text = HEADER_TEMPLATE + vis_models + static_obs + FOOTER
        f_obs = WORLDS_SRC / f'obstacles_{reg}.world'
        f_obs.write_text(obs_text)
        print(f"✅ Generated: {f_obs.name}")
        
        # 3. obstacles_dynamic_<region>.world (Skema 4 & 5 - Statis + Dinamis Pola X)
        obs_dyn_text = HEADER_TEMPLATE + vis_models + static_obs + DYNAMIC_OBS_MODELS + FOOTER
        f_obs_dyn = WORLDS_SRC / f'obstacles_dynamic_{reg}.world'
        f_obs_dyn.write_text(obs_dyn_text)
        print(f"✅ Generated: {f_obs_dyn.name}")

    # Buat juga default fallback 'empty.world' dan 'obstacles.world'
    (WORLDS_SRC / 'empty.world').write_text((WORLDS_SRC / 'empty_rect.world').read_text())
    (WORLDS_SRC / 'obstacles.world').write_text((WORLDS_SRC / 'obstacles_dynamic_rect.world').read_text())
    print("🎉 Seluruh berkas world Gazebo dengan palet warna profesional berhasil dibangkitkan!")

if __name__ == '__main__':
    main()
