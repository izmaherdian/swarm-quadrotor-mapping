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
        ('share/' + package_name + '/config', ['config/cbf_params.yaml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='izmaherdian',
    maintainer_email='izmaherdian@todo.todo',
    description=('Penghindaran tabrakan CBF-QP untuk swarm quadrotor: rintangan '
                 'statis, rintangan bergerak, dan jarak antar-drone resiprokal '
                 'sebagai constraint dalam satu QP per drone per tick.'),
    license='Apache-2.0',
)
