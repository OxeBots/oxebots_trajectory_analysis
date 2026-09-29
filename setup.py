import os
from glob import glob
from setuptools import find_packages, setup

package_name = 'oxebots_trajectory_analysis'

setup(
    name=package_name,
    version='0.1.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'), glob('launch/*.launch.py')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='carlos',
    maintainer_email='ce.cesc01@gmail.com',
    description='Trajectory generation, benchmark and real-time Matplotlib analysis for OxeBots SSL',
    license='GPLv3',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'trajectory_generator_node = oxebots_trajectory_analysis.trajectory_generator_node:main',
            'trajectory_plotter_node = oxebots_trajectory_analysis.trajectory_plotter_node:main',
        ],
    },
)
