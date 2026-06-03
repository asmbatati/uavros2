import os
from glob import glob
from setuptools import setup, find_packages

package_name = 'uav_gz_sim'


def recursive_data_files(directory):
    return [
        (os.path.join('share', package_name, root),
         [os.path.join(root, file) for file in files])
        for root, _, files in os.walk(directory) if files
    ]


setup(
    name=package_name,
    version='0.1.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        *recursive_data_files('config'),
        *recursive_data_files('models'),
        *recursive_data_files('launch'),
        *recursive_data_files('arms'),
        *recursive_data_files('docs'),
        *recursive_data_files('scripts'),
        (os.path.join('share', package_name, 'rviz'), glob('rviz/*.rviz')),
        *([(os.path.join('share', package_name, 'worlds'), glob('worlds/*'))]
          if os.path.exists('worlds') and glob('worlds/*') else []),
    ],
    install_requires=['setuptools'],
    extras_require={'test': ['pytest']},
    zip_safe=True,
    maintainer='Abdulrahman S. Al-Batati',
    maintainer_email='asmalbatati@hotmail.com',
    description=(
        'Multi-simulator UAV + floating-base manipulator simulation framework. '
        'PX4 + Gazebo primary; Webots, MuJoCo, Isaac, PyBullet, Genesis via '
        'dispatcher launch system.'
    ),
    license='MIT',
    entry_points={
        'console_scripts': [
            'execute_random_trajectories = uav_gz_sim.execute_random_trajectories_node:main',
            'tf_relay = uav_gz_sim.tf_relay:main',
            'trajectory_publisher = uav_gz_sim.gt_trajectory_publisher:main',
            'adaptive_image_stitcher = uav_gz_sim.adaptive_image_stitcher:main',
            'sim_control_bridge = uav_gz_sim.sim_control_bridge:main',
            'sensor_relay = uav_gz_sim.sensor_relay:main',
        ],
    },
)
