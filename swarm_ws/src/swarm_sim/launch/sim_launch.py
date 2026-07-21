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
    
    # Daftarkan folder models agar Gazebo bisa mendeteksinya
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
        default_value='7',
        description='Jumlah drone dalam swarm (1 sampai 7)'
    )
    controller_arg = DeclareLaunchArgument(
        'controller',
        default_value='pid_lqr_node',
        description='Kontroler low-level: pid_lqr_node atau pid_hinf_node'
    )
    use_mid_level_arg = DeclareLaunchArgument(
        'use_mid_level',
        default_value='true',
        description='Jalankan node mid-level collision avoidance (ONNX)'
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
    rviz_arg = DeclareLaunchArgument(
        'rviz',
        default_value='true',
        description='Launch RViz2 for 3D visualization'
    )
    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        arguments=['-d', rviz_config_file],
        condition=IfCondition(LaunchConfiguration('rviz')),
        output='screen'
    )

    # 5. Bangun Arsitektur Kontrol Swarm Secara Dinamis (1 sampai 7 drone)
    swarm_nodes = []
    max_drones = 7
    spacing = 2.0  # Jarak antar drone (meter)
    
    for i in range(1, max_drones + 1):
        drone_condition = IfCondition(
            PythonExpression([f"{i} <= ", LaunchConfiguration('num_drones')])
        )
        
        # Formasi sejajar di sepanjang sumbu Y (misal N=7: -6m, -4m, -2m, 0m, 2m, 4m, 6m)
        y_pos = float((i - 4.0) * spacing)

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
        
        # C. Low-Level Controller (PID-LQR / PID-Hinf) dengan Parameter drone_id
        controller_node = Node(
            package='swarm_low_level',
            executable=LaunchConfiguration('controller'),
            name=f'controller_iris_{i}',
            parameters=[
                {'drone_id': i},
                {'log_dir': results_dir}
            ],
            condition=drone_condition,
            output='screen'
        )
        swarm_nodes.append(controller_node)
        
        # D. Mid-Level AI Obstacle Avoidance dengan Parameter drone_id
        mid_level_condition = IfCondition(
            PythonExpression([
                f"{i} <= ", LaunchConfiguration('num_drones'),
                " and '", LaunchConfiguration('use_mid_level'), "' == 'true'"
            ])
        )
        ai_node = Node(
            package='swarm_mid_level',
            executable='collision_avoidance_node',
            name=f'ai_iris_{i}',
            parameters=[
                {'drone_id': i}
            ],
            condition=mid_level_condition,
            output='screen'
        )
        swarm_nodes.append(ai_node)
        
    launch_entities = [
        set_env,
        headless_arg,
        rviz_arg,
        num_drones_arg,
        controller_arg,
        use_mid_level_arg,
        gz_sim_headless,
        gz_sim_gui,
        rviz_node,
    ] + swarm_nodes

    return LaunchDescription(launch_entities)
