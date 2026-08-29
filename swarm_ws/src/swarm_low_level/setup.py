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
        # Dipasang agar PlantModel (swarm_mid_level) dapat menemukannya lewat
        # ament share saat berjalan dari install/, bukan hanya dari src/.
        ('share/' + package_name + '/config', ['config/quadrotor_params.yaml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='user',
    maintainer_email='user@todo.todo',
    description='Low level control for swarm quadrotor',
    license='TODO: License declaration',

    entry_points={
        'console_scripts': [
            'pid_lqr_node = swarm_low_level.pid_lqr_node:main',
            'tf_prefix_node = swarm_low_level.tf_prefix_node:main',
            'pid_hinf_node = swarm_low_level.pid_hinf_node:main',
            'dryden_wind_node = swarm_low_level.dryden_wind_node:main',
        ],
    },
)
