from setuptools import find_packages, setup

package_name = 'swarm_mid_level'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name + '/models', [
            'models/ppo_lidar_avoidance.onnx',
            'models/ppo_lidar_avoidance.onnx.data',
            'models/ppo_lidar_avoidance.zip'
        ]),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='user',
    maintainer_email='user@todo.todo',
    description='Mid level AI control and obstacle avoidance for swarm quadrotor',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'collision_avoidance_node = swarm_mid_level.collision_avoidance_node:main',
        ],
    },
)
