import os
import yaml
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import ExecuteProcess, IncludeLaunchDescription, DeclareLaunchArgument
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution, PythonExpression
from launch_ros.substitutions import FindPackageShare
from launch_ros.actions import Node

'''
Launch script for [TEST ID: t000] DAQ Mode automated test.

IMPORTANT: This launch file is designed to run ALONGSIDE the DEMO_DAY bringup.
Run the 7 DEMO_DAY steps first (pre_slam, zed, lidar, slam, lines, nav2, rviz),
then launch this in an 8th terminal to add DAQ data collection.

This launch file ONLY starts nodes not already covered by DEMO_DAY:
  - gps_publisher         (GPS fix data)
  - electrical_publisher   (voltage/current/power from INA226)
  - data_publisher         (DAQ data collection to CSV)
  - t000_automator         (A button start/stop control)

It does NOT start odom, ZED, control, or joy — those are already
running from pre_slam.launch.py and run-zed.sh.
'''

def generate_launch_description():

    # Launch arguments
    use_sim_time = DeclareLaunchArgument(
        'use_sim_time',
        default_value='false',
        description='Use simulation (Gazebo) clock if true'
    )

    # Package directories
    autonav_testing_share = get_package_share_directory('autonav_automated_testing')
    electrical_share = FindPackageShare('autonav_electrical_publisher')

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
        topics_to_monitor = data_publisher_params.get('t000', [])

    # Data Publisher Node — collects all configured topics into CSV
    data_publisher_node = Node(
        package='autonav_automated_testing',
        executable='data_publisher',
        name='data_publisher_node',
        output='screen',
        parameters=[{
            'topics_to_monitor': list(topics_to_monitor),
            'test_id': 't000',
            'use_sim_time': LaunchConfiguration('use_sim_time')
        }]
    )

    # GPS Handler Node — publishes GPS fix data to /gps_fix
    # (not started by any DEMO_DAY step)
    gps_handler_node = Node(
        package='gps_handler',
        executable='gps_publisher',
        name='gps_publisher_node',
        output='screen',
        parameters=[{
            'use_sim_time': LaunchConfiguration('use_sim_time'),
            'gps_port': '/dev/ttyUSB0'
        }]
    )

    # Electrical Publisher Node — voltage/current/power from INA226
    # (not started by any DEMO_DAY step)
    electrical_publisher_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution([electrical_share, 'launch', 'electrical_publisher.launch.py'])
        )
    )

    # Execute the test automator script (A button start/stop)
    automater_script_path = os.path.join(
        autonav_testing_share,
        'src',
        'automators',
        't000_automator.py'
    )
    test_automater = ExecuteProcess(
        cmd=[
            'python3',
            automater_script_path,
            '--ros-args',
            '-p',
            PythonExpression([
                "'use_sim_time:=' + '", LaunchConfiguration('use_sim_time'), "'"
            ])
        ],
        output='screen',
        name='t000_automater'
    )

    return LaunchDescription([
        use_sim_time,
        # Nodes not covered by DEMO_DAY
        gps_handler_node,
        electrical_publisher_launch,
        # Data collection
        data_publisher_node,
        # Automator
        test_automater
    ])
