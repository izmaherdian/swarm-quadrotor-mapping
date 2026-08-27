import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, AppendEnvironmentVariable, DeclareLaunchArgument
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch.conditions import IfCondition, UnlessCondition
from launch_ros.actions import Node

def generate_launch_description():
    pkg_swarm_sim = get_package_share_directory('swarm_sim')
    pkg_ros_gz_sim = get_package_share_directory('ros_gz_sim')
    
    world_file = os.path.join(pkg_swarm_sim, 'worlds', 'empty.world')
    model_dir = os.path.join(pkg_swarm_sim, 'models')
    
    # Daftarkan folder models agar Gazebo bisa mendeteksi mesh dan model
    set_env = AppendEnvironmentVariable(
        'GZ_SIM_RESOURCE_PATH',
        model_dir
    )
    
    # Deklarasi Arguments Launch
    headless_arg = DeclareLaunchArgument(
        'headless',
        default_value='false',
        description='Jalankan Gazebo tanpa GUI (headless mode)'
    )
    rviz_arg = DeclareLaunchArgument(
        'rviz',
        default_value='true',
        description='Jalankan RViz2 untuk visualisasi pemetaan'
    )

    # 1. Launch Gazebo Server & Client (GUI / Headless)
    gz_sim_headless = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_ros_gz_sim, 'launch', 'gz_sim.launch.py')),
        launch_arguments={'gz_args': f'-r -s "{world_file}"'}.items(),
        condition=IfCondition(LaunchConfiguration('headless'))
    )
    
    gui_config_file = os.path.join(pkg_swarm_sim, 'config', 'gazebo_gui.config')
    gz_sim_gui = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_ros_gz_sim, 'launch', 'gz_sim.launch.py')),
        launch_arguments={'gz_args': f'-r --gui-config "{gui_config_file}" "{world_file}"'}.items(),
        condition=UnlessCondition(LaunchConfiguration('headless'))
    )
    
    # 2. RViz2 Node dengan Konfigurasi Lengkap
    rviz_config_file = os.path.join(pkg_swarm_sim, 'rviz', 'swarm.rviz')
    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        arguments=['-d', rviz_config_file],
        condition=IfCondition(LaunchConfiguration('rviz')),
        output='screen'
    )

    # 3. Global Clock Bridge (Gazebo Sim Clock -> ROS 2 /clock)
    clock_bridge = Node(
        package='ros_gz_bridge',
        executable='parameter_bridge',
        name='global_clock_bridge',
        arguments=['/clock@rosgraph_msgs/msg/Clock[gz.msgs.Clock'],
        output='screen'
    )

    # 4. Static Transform Publisher (world -> swarm_world)
    tf_world_node = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name='static_tf_world_publisher',
        arguments=['--x', '0', '--y', '0', '--z', '0', '--yaw', '0', '--pitch', '0', '--roll', '0', '--frame-id', 'world', '--child-frame-id', 'swarm_world']
    )

    launch_entities = [
        set_env,
        headless_arg,
        rviz_arg,
        gz_sim_headless,
        gz_sim_gui,
        rviz_node,
        clock_bridge,
        tf_world_node,
    ]

    return LaunchDescription(launch_entities)
