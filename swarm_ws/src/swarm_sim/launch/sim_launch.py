import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, AppendEnvironmentVariable
from launch.launch_description_sources import PythonLaunchDescriptionSource
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
    
    # Jalankan Gazebo (-r untuk auto-run, -s untuk jalankan server/headless jika tanpa GUI)
    gz_sim = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_ros_gz_sim, 'launch', 'gz_sim.launch.py')),
        launch_arguments={'gz_args': f'-r "{world_file}"'}.items(),
    )
    
    # Spawn drone iris_base ke dalam dunia Gazebo
    spawn_drone = Node(
        package='ros_gz_sim',
        executable='create',
        arguments=[
            '-world', 'empty_world',
            '-name', 'iris_1',
            '-file', os.path.join(model_dir, 'iris_base', 'model.sdf'),
            '-x', '0.0',
            '-y', '0.0',
            '-z', '5.0'
        ],
        output='screen'
    )
    
    # Bridge Odometry dari Gazebo ke ROS 2
    bridge = Node(
        package='ros_gz_bridge',
        executable='parameter_bridge',
        arguments=[
            '/model/iris_1/odometry@nav_msgs/msg/Odometry[gz.msgs.Odometry'
        ],
        output='screen'
    )
    
    # Node Logger/Kontroler PID-LQR
    pid_node = Node(
        package='swarm_low_level',
        executable='pid_lqr_node',
        name='pid_lqr_node',
        output='screen'
    )
    
    return LaunchDescription([
        set_env,
        gz_sim,
        spawn_drone,
        bridge,
        pid_node
    ])
