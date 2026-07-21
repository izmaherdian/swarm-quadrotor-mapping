import os
import glob
from setuptools import find_packages, setup

package_name = 'swarm_low_level'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        # Install config YAML ke share/swarm_low_level/config/
        (os.path.join('share', package_name, 'config'),
            glob.glob('config/*.yaml')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='user',
    maintainer_email='user@todo.todo',
    description='Low level control for swarm quadrotor',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'pid_lqr_node = swarm_low_level.pid_lqr_node:main',
            'pid_hinf_node = swarm_low_level.pid_hinf_node:main',
        ],
    },
)
