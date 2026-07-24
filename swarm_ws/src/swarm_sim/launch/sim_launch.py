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
    results_base_arg = DeclareLaunchArgument(
        'results_base',
        default_value='multi_agent',
        description='Subfolder di results/ untuk menyimpan CSV (e.g. multi_agent, single_agent)'
    )

    # 3. Launch Gazebo Server & Client (GUI/Headless)
    gz_args_headless = f'-r -s "{world_file}"'
    gz_args_gui = f'-r "{world_file}"'
    
    gz_sim_headless = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_ros_gz_sim, 'launch', 'gz_sim.launch.py')),
        launch_arguments={'gz_args': f'-r -s "{world_file}"'}.items(),
        condition=IfCondition(LaunchConfiguration('headless'))
    )
    
    gz_sim_gui = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_ros_gz_sim, 'launch', 'gz_sim.launch.py')),
        launch_arguments={'gz_args': f'-r "{world_file}"'}.items(),
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

    # 5. Bangun Arsitektur Kontrol Swarm Secara Dinamis (1 sampai 7 drone)
    pkg_share = get_package_share_directory('swarm_sim')
    ws_root = os.path.abspath(os.path.join(pkg_share, '../../../../'))
    import sys
    _results_base = 'multi_agent'
    for arg in sys.argv:
        if arg.startswith('results_base:='):
            _results_base = arg.split(':=', 1)[1]
            break
    base_results_dir = os.path.join(ws_root, 'src', 'swarm_sim', 'results', _results_base)
    config_dir = os.path.join(ws_root, 'src', 'swarm_low_level', 'config')

    # Buat kedua subfolder hasil simulasi sekaligus
    results_lqr  = os.path.join(base_results_dir, 'pid_lqr')
    results_hinf = os.path.join(base_results_dir, 'pid_hinf')
    os.makedirs(results_lqr,  exist_ok=True)
    os.makedirs(results_hinf, exist_ok=True)

    # Pilih subfolder berdasarkan controller yang dipakai
    _ctrl = None
    for arg in sys.argv:
        if arg.startswith('controller:='):
            _ctrl = arg.split(':=', 1)[1]
            break
    if _ctrl and 'hinf' in _ctrl:
        results_dir = results_hinf
    else:
        results_dir = results_lqr

    swarm_nodes = []
    max_drones = 7
    spacing = 2.0  # Jarak antar drone (meter)
    
    for i in range(1, max_drones + 1):
        drone_condition = IfCondition(
            PythonExpression([f"{i} <= ", LaunchConfiguration('num_drones')])
        )
        
        # Per-drone bridge node for complete topic isolation
        bridge_args = [
            '/clock@rosgraph_msgs/msg/Clock[gz.msgs.Clock',
            f'/model/iris_{i}/odometry@nav_msgs/msg/Odometry[gz.msgs.Odometry',
            f'/iris_{i}/command/motor_speed@actuator_msgs/msg/Actuators]gz.msgs.Actuators',
            f'/world/swarm_world/model/iris_{i}/link/base_link/sensor/gpu_lidar/scan@sensor_msgs/msg/LaserScan[gz.msgs.LaserScan',
            f'/model/iris_{i}/pose@tf2_msgs/msg/TFMessage[gz.msgs.Pose_V',
        ]
        bridge_remaps = [
            (f'/model/iris_{i}/odometry', f'/iris_{i}/odometry'),
            (f'/world/swarm_world/model/iris_{i}/link/base_link/sensor/gpu_lidar/scan', f'/iris_{i}/lidar_scan'),
            (f'/model/iris_{i}/pose', '/tf'),
        ]
        swarm_nodes.append(Node(
            package='ros_gz_bridge',
            executable='parameter_bridge',
            name=f'bridge_iris_{i}',
            arguments=bridge_args,
            remappings=bridge_remaps,
            condition=drone_condition,
            output='screen'
        ))
        
        y_pos = float((i - 4.0) * spacing)

        spawn_node = Node(
            package='ros_gz_sim',
            executable='create',
            name=f'spawn_iris_{i}',
            arguments=[
                '-world', 'swarm_world',
                '-name', f'iris_{i}',
                '-file', os.path.join(model_dir, f'iris_{i}', 'model.sdf'),
                '-x', '0.0',
                '-y', str(y_pos),
                '-z', '0.01'
            ],
            condition=drone_condition,
            output='screen'
        )
        swarm_nodes.append(spawn_node)
        
        # C. Low-Level Controller (PID-LQR / PID-Hinf) dengan Parameter drone_id
        controller_node = Node(
            package='swarm_low_level',
            executable=LaunchConfiguration('controller'),
            name=f'controller_iris_{i}',
            namespace=f'iris_{i}',
            parameters=[
                {'drone_id': i},
                {'log_dir': results_dir},
                {'config_dir': config_dir}
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
        
        # E. TF Prefix Node untuk mencegah bentrok frame base_link antar drone
        tf_prefix = Node(
            package='swarm_low_level',
            executable='tf_prefix_node',
            name=f'tf_prefix_iris_{i}',
            parameters=[
                {'drone_id': i}
            ],
            condition=drone_condition,
            output='screen'
        )
        swarm_nodes.append(tf_prefix)
    tf_node = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        arguments=['--x', '0', '--y', '0', '--z', '0', '--yaw', '0', '--pitch', '0', '--roll', '0', '--frame-id', 'world', '--child-frame-id', 'swarm_world']
    )


        
    launch_entities = [
        set_env,
        headless_arg,
        rviz_arg,
        num_drones_arg,
        controller_arg,
        use_mid_level_arg,
        results_base_arg,
        gz_sim_headless,
        gz_sim_gui,
        rviz_node,
        tf_node,
    ] + swarm_nodes

    return LaunchDescription(launch_entities)
