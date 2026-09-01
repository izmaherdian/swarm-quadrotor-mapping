import os
import sys
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, AppendEnvironmentVariable
from launch.substitutions import LaunchConfiguration, PythonExpression
from launch.conditions import IfCondition
from launch_ros.actions import Node

def generate_launch_description():
    pkg_swarm_sim = get_package_share_directory('swarm_sim')
    model_dir = os.path.join(pkg_swarm_sim, 'models')
    
    set_env = AppendEnvironmentVariable(
        'GZ_SIM_RESOURCE_PATH',
        model_dir
    )

    num_drones_arg = DeclareLaunchArgument(
        'num_drones',
        default_value='1',
        description='Jumlah drone yang di-spawn (1 sampai 7)'
    )
    controller_arg = DeclareLaunchArgument(
        'controller',
        default_value='pid_lqr_node',
        description='Kontroler low-level: pid_lqr_node atau pid_hinf_node'
    )
    use_mid_level_arg = DeclareLaunchArgument(
        'use_mid_level',
        default_value='true',
        description='Jalankan node mid-level collision avoidance'
    )
    results_base_arg = DeclareLaunchArgument(
        'results_base',
        default_value='single_agent',
        description='Subfolder di results/ untuk menyimpan CSV (e.g. single_agent, multi_agent)'
    )
    spawn_x_arg = DeclareLaunchArgument(
        'spawn_x',
        default_value='-5.5',
        description='Posisi X awal spawn iris_1'
    )
    spawn_y_arg = DeclareLaunchArgument(
        'spawn_y',
        default_value='-5.5',
        description='Posisi Y awal spawn iris_1'
    )

    pkg_share = get_package_share_directory('swarm_sim')
    ws_root = os.path.abspath(os.path.join(pkg_share, '../../../../'))
    
    _results_base = 'single_agent'
    _spawn_x = '-5.5'
    _spawn_y = '-5.5'
    for arg in sys.argv:
        if arg.startswith('results_base:='):
            _results_base = arg.split(':=', 1)[1]
        elif arg.startswith('spawn_x:='):
            _spawn_x = arg.split(':=', 1)[1]
        elif arg.startswith('spawn_y:='):
            _spawn_y = arg.split(':=', 1)[1]
    if os.path.isabs(_results_base):
        base_results_dir = _results_base
    elif _results_base.startswith('results/'):
        base_results_dir = os.path.join(ws_root, _results_base)
    else:
        base_results_dir = os.path.join(ws_root, 'src', 'swarm_sim', 'results', _results_base)
    config_dir = os.path.join(ws_root, 'src', 'swarm_low_level', 'config')

    results_lqr  = os.path.join(base_results_dir, 'pid_lqr')
    results_hinf = os.path.join(base_results_dir, 'pid_hinf')
    os.makedirs(results_lqr,  exist_ok=True)
    os.makedirs(results_hinf, exist_ok=True)

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
    spacing = 2.0

    for i in range(1, max_drones + 1):
        drone_condition = IfCondition(
            PythonExpression([f"{i} <= ", LaunchConfiguration('num_drones')])
        )
        
        # 1. Per-drone ROS-Gazebo Parameter Bridge
        bridge_args = [
            f'/model/iris_{i}/odometry@nav_msgs/msg/Odometry[gz.msgs.Odometry',
            f'/iris_{i}/command/motor_speed@actuator_msgs/msg/Actuators]gz.msgs.Actuators',
            f'/world/swarm_world/model/iris_{i}/link/base_link/sensor/gpu_lidar/scan@sensor_msgs/msg/LaserScan[gz.msgs.LaserScan',
        ]
        bridge_remaps = [
            (f'/model/iris_{i}/odometry', f'/iris_{i}/odometry'),
            (f'/world/swarm_world/model/iris_{i}/link/base_link/sensor/gpu_lidar/scan', f'/iris_{i}/lidar_scan'),
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
        
        # 2. Dynamic Model Spawning into Running Gazebo World
        _num_drones_int = 7
        _spawn_coords = {}
        for idx in range(1, 8):
            _spawn_coords[f'x{idx}'] = None
            _spawn_coords[f'y{idx}'] = None

        for _a in sys.argv:
            if _a.startswith('num_drones:='):
                try:
                    _num_drones_int = int(_a.split(':=', 1)[1])
                except ValueError:
                    pass
            for idx in range(1, 8):
                if _a.startswith(f'spawn_x{idx}:='):
                    _spawn_coords[f'x{idx}'] = _a.split(':=', 1)[1]
                elif _a.startswith(f'spawn_y{idx}:='):
                    _spawn_coords[f'y{idx}'] = _a.split(':=', 1)[1]

        if _spawn_coords.get(f'x{i}') is not None and _spawn_coords.get(f'y{i}') is not None:
            x_pos_str = _spawn_coords[f'x{i}']
            y_pos_str = _spawn_coords[f'y{i}']
        elif _num_drones_int == 1:
            x_pos_str = _spawn_coords.get('x1') or _spawn_x
            y_pos_str = _spawn_coords.get('y1') or _spawn_y
        elif _num_drones_int == 2:
            x_pos_str = _spawn_coords.get('x1') or ('-6.0' if i == 1 else '6.0')
            y_pos_str = _spawn_coords.get('y1') or '0.0'
        else:
            default_positions = {
                1: ('-3.0', '-18.0'),
                2: ('-1.0', '-18.0'),
                3: ('1.0',  '-18.0'),
                4: ('3.0',  '-18.0'),
                5: ('-2.0', '-16.5'),
                6: ('0.0',  '-16.5'),
                7: ('2.0',  '-16.5'),
            }
            x_pos_str, y_pos_str = default_positions.get(i, ('0.0', str(float((i - 4.0) * spacing))))

        spawn_node = Node(
            package='ros_gz_sim',
            executable='create',
            name=f'spawn_iris_{i}',
            arguments=[
                '-world', 'swarm_world',
                '-name', f'iris_{i}',
                '-file', os.path.join(model_dir, f'iris_{i}', 'model.sdf'),
                '-x', x_pos_str,
                '-y', y_pos_str,
                '-z', '0.01',
                '-Y', '1.57079632679'
            ],
            condition=drone_condition,
            output='screen'
        )
        swarm_nodes.append(spawn_node)
        
        # 3. Low-Level Controller (PID-LQR / PID-Hinf) dengan Auto-Takeoff ke Z=2.0m
        controller_node = Node(
            package='swarm_low_level',
            executable=LaunchConfiguration('controller'),
            name=f'controller_iris_{i}',
            namespace=f'iris_{i}',
            parameters=[
                {'drone_id': i},
                {'log_dir': results_dir},
                {'config_dir': config_dir},
                {'use_sim_time': True}
            ],
            condition=drone_condition,
            output='screen'
        )
        swarm_nodes.append(controller_node)
        
        # 4. Mid-Level Collision Avoidance Node
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
                {'drone_id': i},
                {'max_speed': 1.0},
                {'num_drones': LaunchConfiguration('num_drones')},
                {'use_sim_time': True}
            ],
            condition=mid_level_condition,
            output='screen'
        )
        swarm_nodes.append(ai_node)
        


    # 6. Global Clock Bridge (Gazebo Sim Clock -> ROS 2 /clock) & Dynamic Obstacle Bridges
    clock_bridge = Node(
        package='ros_gz_bridge',
        executable='parameter_bridge',
        name='global_clock_bridge',
        arguments=[
            '/clock@rosgraph_msgs/msg/Clock[gz.msgs.Clock',
            '/model/dynamic_obs_1/cmd_vel@geometry_msgs/msg/Twist]gz.msgs.Twist',
            '/model/dynamic_obs_2/cmd_vel@geometry_msgs/msg/Twist]gz.msgs.Twist',
            '/model/dynamic_obs_1/odometry@nav_msgs/msg/Odometry[gz.msgs.Odometry',
            '/model/dynamic_obs_2/odometry@nav_msgs/msg/Odometry[gz.msgs.Odometry',
        ],
        output='screen'
    )
    swarm_nodes.append(clock_bridge)

    # 7. Static Transform Publisher (world -> swarm_world)
    tf_world_node = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name='static_tf_world_publisher',
        arguments=['--x', '0', '--y', '0', '--z', '0', '--yaw', '0', '--pitch', '0', '--roll', '0', '--frame-id', 'world', '--child-frame-id', 'swarm_world']
    )
    swarm_nodes.append(tf_world_node)

    # 8. Dryden Wind Turbulence Generator Node
    enable_wind_arg = DeclareLaunchArgument(
        'enable_wind',
        default_value='false',
        description='Aktifkan Dryden Wind Turbulence generator'
    )

    wind_node = Node(
        package='swarm_low_level',
        executable='dryden_wind_node',
        name='dryden_wind_node',
        parameters=[{'use_sim_time': True}],
        condition=IfCondition(LaunchConfiguration('enable_wind')),
        output='screen'
    )
    swarm_nodes.append(wind_node)

    launch_entities = [
        set_env,
        num_drones_arg,
        controller_arg,
        use_mid_level_arg,
        enable_wind_arg,
        results_base_arg,
        spawn_x_arg,
        spawn_y_arg,
    ] + swarm_nodes

    return LaunchDescription(launch_entities)
