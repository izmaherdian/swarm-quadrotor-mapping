import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, AppendEnvironmentVariable, DeclareLaunchArgument, EmitEvent
from launch.events import Shutdown
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PythonExpression
from launch.conditions import IfCondition, UnlessCondition
from launch_ros.actions import Node

def generate_launch_description():
    pkg_swarm_sim = get_package_share_directory('swarm_sim')
    pkg_ros_gz_sim = get_package_share_directory('ros_gz_sim')
    
    world_file = os.path.join(pkg_swarm_sim, 'worlds', 'empty.world')
    model_dir = os.path.join(pkg_swarm_sim, 'models')
    
    # 1. Environment Variable untuk resource Gazebo
    set_env = AppendEnvironmentVariable(
        'GZ_SIM_RESOURCE_PATH',
        model_dir
    )
    
    # 2. Deklarasi Arguments Launch
    headless_arg = DeclareLaunchArgument(
        'headless',
        default_value='false',
        description='Jalankan Gazebo tanpa GUI (headless mode)'
    )
    rviz_arg = DeclareLaunchArgument(
        'rviz',
        default_value='true',
        description='Jalankan RViz2 untuk visualisasi'
    )
    num_drones_arg = DeclareLaunchArgument(
        'num_drones',
        default_value='3',
        description='Jumlah drone dalam swarm (1 sampai 7)'
    )
    controller_arg = DeclareLaunchArgument(
        'controller',
        default_value='pid_lqr_node',
        description='Kontroler low-level: pid_lqr_node atau pid_hinf_node'
    )

    # 3. Launch Gazebo Server & Client (GUI/Headless)
    gz_args_headless = f'-r -s "{world_file}"'
    gz_args_gui = f'-r "{world_file}"'
    
    gz_sim_headless = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_ros_gz_sim, 'launch', 'gz_sim.launch.py')),
        launch_arguments={'gz_args': gz_args_headless}.items(),
        condition=IfCondition(LaunchConfiguration('headless'))
    )
    
    gz_sim_gui = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_ros_gz_sim, 'launch', 'gz_sim.launch.py')),
        launch_arguments={'gz_args': gz_args_gui}.items(),
        condition=UnlessCondition(LaunchConfiguration('headless'))
    )
    
    # 4. RViz2 Node
    rviz_config_file = os.path.join(pkg_swarm_sim, 'rviz', 'swarm.rviz')
    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        arguments=['-d', rviz_config_file],
        condition=IfCondition(LaunchConfiguration('rviz')),
        output='screen'
    )

    # 5. Bangun Arsitektur Kontrol Swarm Secara Dinamis
    # Kita buat list untuk menampung seluruh Node drone yang di-looping
    swarm_nodes = []
    
    # Max drone yang disupport oleh launch ini adalah 7 (sesuai spesifikasi geodesi)
    max_drones = 7
    spacing = 2.0 # Jarak antar drone saat spawn (meter)
    
    # Untuk mendukung pencabangan dinamis dari parameter num_drones di LaunchDescription,
    # kita mengevaluasi ekspresi loop dengan batasan PythonExpression
    for i in range(1, max_drones + 1):
        # Hitung posisi Y awal agar drone tersebar simetris di sekitar Y=0
        # Formula: y = (i - (N+1)/2) * spacing
        # Karena i adalah integer statik loop, kita evaluasi kondisi spawn berdasarkan num_drones
        drone_condition = IfCondition(
            PythonExpression([f"{i} <= ", LaunchConfiguration('num_drones')])
        )
        
        # Hitung y_pos secara statik untuk drone ke-i
        # Misal N=3: drone_1 (y=-2), drone_2 (y=0), drone_3 (y=2)
        # Kita hitung berdasarkan perkiraan N=LaunchConfiguration('num_drones')
        # Namun untuk presisi koordinat awal spawn fisik Gazebo, kita gunakan formula linear sederhana:
        # Kita offset posisi berdasarkan indeks i
        y_pos = float((i - 2.0) * spacing) # iris_1 di -2.0, iris_2 di 0.0, iris_3 di 2.0, dst.

        # A. Spawn Node
        spawn_node = Node(
            package='ros_gz_sim',
            executable='create',
            name=f'spawn_iris_{i}',
            arguments=[
                '-world', 'swarm_world',
                '-name', f'iris_{i}',
                '-file', os.path.join(model_dir, 'iris_base', 'model.sdf'),
                '-x', '0.0',
                '-y', str(y_pos),
                '-z', '0.01'
            ],
            condition=drone_condition,
            output='screen'
        )
        swarm_nodes.append(spawn_node)
        
        # B. Bridge Node (Odometry, Actuators, LiDAR, TF)
        bridge_node = Node(
            package='ros_gz_bridge',
            executable='parameter_bridge',
            name=f'bridge_iris_{i}',
            arguments=[
                f'/model/iris_{i}/odometry@nav_msgs/msg/Odometry[gz.msgs.Odometry',
                f'/iris_{i}/command/motor_speed@actuator_msgs/msg/Actuators]gz.msgs.Actuators',
                f'/world/swarm_world/model/iris_{i}/link/base_link/sensor/gpu_lidar/scan@sensor_msgs/msg/LaserScan[gz.msgs.LaserScan',
                f'/model/iris_{i}/pose@tf2_msgs/msg/TFMessage[gz.msgs.Pose_V'
            ],
            remappings=[
                (f'/world/swarm_world/model/iris_{i}/link/base_link/sensor/gpu_lidar/scan', f'/iris_{i}/lidar_scan'),
                (f'/model/iris_{i}/pose', '/tf')
            ],
            condition=drone_condition,
            output='screen'
        )
        swarm_nodes.append(bridge_node)
        
        # C. Low-Level Controller (LQR / H-infinity) dengan Remapping
        controller_node = Node(
            package='swarm_low_level',
            executable=LaunchConfiguration('controller'),
            name=f'controller_iris_{i}',
            remappings=[
                ('/model/iris_1/odometry', f'/model/iris_{i}/odometry'),
                ('/iris_1/command/motor_speed', f'/iris_{i}/command/motor_speed'),
                ('/iris_1/target_pose', f'/iris_{i}/target_pose')
            ],
            condition=drone_condition,
            output='screen'
        )
        swarm_nodes.append(controller_node)
        
        # D. Mid-Level AI Obstacle Avoidance dengan Remapping
        ai_node = Node(
            package='swarm_mid_level',
            executable='collision_avoidance_node',
            name=f'ai_iris_{i}',
            remappings=[
                ('/model/iris_1/odometry', f'/model/iris_{i}/odometry'),
                ('/iris_1/waypoint', f'/iris_{i}/waypoint'),
                ('/iris_1/target_pose', f'/iris_{i}/target_pose'),
                ('/lidar_scan', f'/iris_{i}/lidar_scan')
            ],
            condition=drone_condition,
            output='screen'
        )
        swarm_nodes.append(ai_node)
        
        # E. High-Level P2P Heartbeat
        heartbeat_node = Node(
            package='swarm_high_level',
            executable='heartbeat_p2p',
            name=f'heartbeat_iris_{i}',
            parameters=[
                {'drone_id': i},
                {'num_drones': LaunchConfiguration('num_drones')}
            ],
            condition=drone_condition,
            output='screen'
        )
        swarm_nodes.append(heartbeat_node)
        
        # F. High-Level Voronoi Cell Partition
        voronoi_node = Node(
            package='swarm_high_level',
            executable='voronoi_node',
            name=f'voronoi_iris_{i}',
            parameters=[
                {'drone_id': i},
                {'num_drones': LaunchConfiguration('num_drones')},
                {'x_min': 0.0},
                {'x_max': 10.0},
                {'y_min': -5.0},
                {'y_max': 5.0}
            ],
            condition=drone_condition,
            output='screen'
        )
        swarm_nodes.append(voronoi_node)
        
        # G. High-Level Bezier Path Smoother
        bezier_node = Node(
            package='swarm_high_level',
            executable='bezier_path',
            name=f'bezier_iris_{i}',
            parameters=[
                {'drone_id': i}
            ],
            condition=drone_condition,
            output='screen'
        )
        swarm_nodes.append(bezier_node)

    # 6. Susun LaunchDescription Akhir
    launch_entities = [
        set_env,
        headless_arg,
        rviz_arg,
        num_drones_arg,
        controller_arg,
        gz_sim_headless,
        gz_sim_gui,
        rviz_node
    ] + swarm_nodes

    return LaunchDescription(launch_entities)
