import subprocess
import sys
import yaml
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.substitutions import FindPackageShare
from launch_ros.actions import Node

'''
Launch script for [TEST ID: 001] automated test script.
Launching this script will start the test.
'''

def generate_launch_description():

    # Grab the YAML that stores all the Topic names that we need to subscribe to for this test.
    test_data_config = os.path.join(
        get_package_share_directory('autonav_automated_testing'),
        'config',
        'testing_data_collection_setter.yaml'
    )
    with open( test_data_config, 'r') as f:
        all_params = yaml.safe_load(f)
        topics = all_params.get('t001', {})  

    data_publisher_node = Node(
        package='data_publisher',
        executable='data_publisher',
        name='data_publisher_node',
        parameters=[topics]
    )

    # Launch the specific python script to automatically start the test.
    # For this test launch the t001_automater.py file.
    subprocess.run([sys.executable, "t001_automater.py"])

    return LaunchDescription([
        data_publisher_node
        ])
