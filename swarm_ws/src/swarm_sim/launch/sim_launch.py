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
    
    # Setup Headless Argument
    headless_arg = DeclareLaunchArgument(
        'headless',
        default_value='false',
        description='Run Gazebo in headless mode (no GUI)'
    )
    
    # Run Gazebo headless (-s) or with GUI
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
    
    # Spawn drone iris_base ke dalam dunia Gazebo
    spawn_drone = Node(
        package='ros_gz_sim',
        executable='create',
        arguments=[
            '-world', 'empty_world',
            '-name', 'iris_1',
            '-file', os.path.join(model_dir, 'iris_base', 'model.sdf'),
            '-x', '1.0',
            '-y', '1.0',
            '-z', '1.0'
        ],
        output='screen'
    )
    
    # Bridge Odometry & Actuators dari/ke Gazebo
    bridge = Node(
        package='ros_gz_bridge',
        executable='parameter_bridge',
        arguments=[
            '/model/iris_1/odometry@nav_msgs/msg/Odometry[gz.msgs.Odometry',
            '/iris_1/command/motor_speed@actuator_msgs/msg/Actuators]gz.msgs.Actuators'
        ],
        output='screen'
    )
    
    # Declare argument for controller type
    controller_arg = DeclareLaunchArgument(
        'controller',
        default_value='pid_lqr_node',
        description='Which controller to run: pid_lqr_node or pid_hinf_node'
    )
    
    # Define the absolute path to the results directory inside the src tree
    pkg_share = get_package_share_directory('swarm_sim')
    # Because pkg_share is usually .../install/swarm_sim/share/swarm_sim, 
    # we go up 4 levels to the workspace root, then into src.
    ws_root = os.path.abspath(os.path.join(pkg_share, '../../../../'))
    results_dir = os.path.join(ws_root, 'src', 'swarm_sim', 'results', 'single_agent')
    os.makedirs(results_dir, exist_ok=True)
    
    # 2. Node untuk Controller (Bisa PID-LQR atau PID-HINF)
    controller_node = Node(
        package='swarm_low_level',
        executable=LaunchConfiguration('controller'),
        name=LaunchConfiguration('controller'),
        output='screen',
        parameters=[{'log_dir': results_dir}],
        on_exit=[EmitEvent(event=Shutdown())]
    )
    
    return LaunchDescription([
        set_env,
        headless_arg,
        controller_arg,
        gz_sim_headless,
        gz_sim_gui,
        spawn_drone,
        bridge,
        controller_node
    ])
