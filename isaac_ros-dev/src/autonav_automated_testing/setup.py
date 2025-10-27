from setuptools import setup
import os
from glob import glob

package_name = 'autonav_automated_testing'

setup(
    name=package_name,
    version='0.1.0',
    packages=[package_name],
    data_files=[
        # Standard ROS2 index paths
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),

        # Install launch files
        (os.path.join('share', package_name, 'launch'), glob('launch/*.launch.py')),

        # Plugin export
        (os.path.join('share', package_name), ['plugin.xml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Your Name',
    maintainer_email='you@example.com',
    description='Automated Testing RQt Plugin for ROS2',
    license='MIT',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            # Optional CLI launcher if you want one
        ],
    },
)