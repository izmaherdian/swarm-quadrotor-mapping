from setuptools import find_packages, setup

package_name = 'swarm_high_level'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='izmaherdian',
    maintainer_email='izmaherdian@todo.todo',
    description='Swarm coordination algorithms: Voronoi, Bezier paths, Heartbeat P2P',
    license='Apache-2.0',

    entry_points={
        'console_scripts': [
            'simulator_kinematics = swarm_high_level.simulator_kinematics:main',
        ],
    },
)
