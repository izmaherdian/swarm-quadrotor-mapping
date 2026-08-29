from glob import glob

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
        ('share/' + package_name + '/config/regions',
            glob('config/regions/*.yaml')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='izmaherdian',
    maintainer_email='izmaherdian@todo.todo',
    description=('Koordinasi swarm: definisi dunia, metrik evaluasi, dan '
                 'pelaporan kuantitatif untuk pemetaan Voronoi-boustrophedon.'),
    license='Apache-2.0',

    entry_points={
        'console_scripts': [
            # Laporan kuantitatif satu/beberapa run untuk tabel paper.
            'swarm_run_report = swarm_high_level.metrics.run_report:main',
        ],
    },
)
