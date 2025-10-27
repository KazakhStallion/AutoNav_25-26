
import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.substitutions import FindPackageShare
from launch_ros.actions import Node

'''
Launch script for [TEST ID: 001] automated test script.
Launching this script will start the test.
'''

def generate_launch_description():

    data_collection_node = Node(
        package='',
        executable='',
        output='',
        parameters=[]
    )

    return LaunchDescription([
        data_collection_node
        ])
