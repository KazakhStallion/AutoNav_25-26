import os
import yaml
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import ExecuteProcess, IncludeLaunchDescription, DeclareLaunchArgument
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare
from launch_ros.actions import Node

'''
Launch script for [TEST ID: t004] Object Detection automated test.
Launching this script will:
1. Start the data_publisher node to collect test data
2. Launch specific Nodes or Launch files related to this test
3. Execute the t004_automater.py script to manage the test
'''

def generate_launch_description():

    # Declare launch arguments
    use_sim_time = DeclareLaunchArgument(
        'use_sim_time',
        default_value='false',
        description='Use simulation (Gazebo) clock if true'
    )

    # Get package directories
    autonav_testing_share = get_package_share_directory('autonav_automated_testing')
    # ===== Test Specific Packages ===== #
    # ================================== #

    # Path to test data configuration file
    test_data_config = os.path.join(
        autonav_testing_share,
        'config',
        'testing_data_collection_setter.yaml'
    )
    
    # Load topics to monitor for this specific test
    with open(test_data_config, 'r') as f:
        all_params = yaml.safe_load(f)
        data_publisher_params = all_params.get('data_publisher', {}).get('ros__parameters', {})
        topics_to_monitor = data_publisher_params.get('t004', [])

    # Data Publisher Node - collects data from specified topics
    data_publisher_node = Node(
        package='autonav_automated_testing',
        executable='data_publisher',
        name='data_publisher_node',
        output='screen',
        parameters=[{
            'topics_to_monitor': list(topics_to_monitor),
            'test_id': 't004',
            'use_sim_time': LaunchConfiguration('use_sim_time')
        }]
    )

    # ===== Lines below here are specific to the t004 test ===== #

    # [NODES] #
    # [LAUNCH FILES] #

    # Execute the test automater script
    # This script handles test orchestration, data collection, and log file generation
    automater_script_path = os.path.join(
        autonav_testing_share,
        'src',
        'automaters',
        't004_automater.py'
    )
    
    test_automater = ExecuteProcess(
        cmd=['python3', automater_script_path],
        output='screen',
        name='t004_automater'
    )
    # ========================================================== #

    return LaunchDescription([
        # Launch arguments
        use_sim_time,
        
        # ===== Specific to test ===== #
        # [NODES] #
        # [LAUNCH FILES] #
        # ============================ #
        
        # Data collection node
        data_publisher_node,
        
        # Test automation script
        test_automater
    ])
