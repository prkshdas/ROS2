import os
from glob import glob
from setuptools import find_packages, setup

package_name = 'imu_tracker'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        # --- THE MAGIC LINES ---
        # 1. Tell colcon to copy all .urdf files from the urdf folder
        (os.path.join('share', package_name, 'urdf'), glob(os.path.join('urdf', '*.urdf'))),
        # 2. Tell colcon to copy all launch files from the launch folder
        (os.path.join('share', package_name, 'launch'), glob(os.path.join('launch', '*launch.[pxy][yma]*'))),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='root',
    maintainer_email='prkshhhdas@gmail.com',
    description='IMU tracking and visualization package',
    license='TODO: License declaration',
    extras_require={
        'test': [
            'pytest',
        ],
    },
    entry_points={
        'console_scripts': [
            # Registers your Python node so ROS 2 can find it
            'serial_bridge = imu_tracker.serial_bridge:main'
        ],
    },
)
