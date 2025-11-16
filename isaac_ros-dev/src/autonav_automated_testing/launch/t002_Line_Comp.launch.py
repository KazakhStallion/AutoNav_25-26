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
Launch script for [TEST ID: t002] Line Compliance automated test.
Launching this script will:
1. Start the data_publisher node to collect test data
2. Launch specific Nodes or Launch files related to this test
3. Execute the t002_automator.py script to manage the test
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
    bringup_share = FindPackageShare('bringup')
    control_share = FindPackageShare('control')
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
        topics_to_monitor = data_publisher_params.get('t002', [])

    # Data Publisher Node - collects data from specified topics
    data_publisher_node = Node(
        package='autonav_automated_testing',
        executable='data_publisher',
        name='data_publisher_node',
        output='screen',
        parameters=[{
            'topics_to_monitor': list(topics_to_monitor),
            'test_id': 't002',
            'use_sim_time': LaunchConfiguration('use_sim_time')
        }]
    )

    # ===== Lines below here are specific to the t002 test ===== #

    # [NODES] #
    # GPS Handler Node - publishes GPS fix data to /gps/fix
    gps_handler_node = Node(
        package='gps_handler',
        executable='gps_publisher',
        name='gps_publisher_node',
        output='screen',
        parameters=[{
            'use_sim_time': LaunchConfiguration('use_sim_time'),
            'gps_port': '/dev/ttyUSB0'  # Adjust based on your GPS device
        }]
    )

    # Line Detection Node - detects white lines for compliance testing
    line_detection_node = Node(
        package='line_detection',
        executable='line_detector',
        name='line_detection_node',
        output='screen',
        parameters=[{
            'use_sim_time': LaunchConfiguration('use_sim_time')
        }]
    )

    # Odometry Handler Node - publishes wheel encoder data to /encoders and /odom
    odom_handler_node = Node(
        package='odom_handler',
        executable='wheel_odometry_publisher',
        name='odom_publisher_node',
        output='screen',
        parameters=[{
            'use_sim_time': LaunchConfiguration('use_sim_time')
        }]
    )

    # [LAUNCH FILES] #
    # Include standard bringup launch file (includes camera, sensors, SLAM)
    # This provides: ZED camera, SICK LiDAR (/scan), TF transforms, SLAM
    standard_bringup = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution([bringup_share, 'launch', 'bringup.launch.py'])
        ),
        launch_arguments={
            'use_sim_time': LaunchConfiguration('use_sim_time')
        }.items()
    )

    # Include control launch file for motor control and /cmd_vel processing
    control_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution([control_share, 'launch', 'control_dev.launch.py'])
        ),
        launch_arguments={
            'use_sim_time': LaunchConfiguration('use_sim_time')
        }.items()
    )

    # Execute the test automater script
    # This script handles test orchestration, data collection, and log file generation
    automater_script_path = os.path.join(
        autonav_testing_share,
        'src',
        'automators',
        't002_automator.py'
    )
    
    # Pass use_sim_time through to the automator process so its ROS clock matches the rest of the system
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
        name='t002_automater'
    )
    # ========================================================== #

    return LaunchDescription([
        # Launch arguments
        use_sim_time,
        
        # ===== Specific to test ===== #
        # [NODES] #
        gps_handler_node,
        odom_handler_node,
        line_detection_node,
        
        # [LAUNCH FILES] #
        standard_bringup,
        # [IMPORTANT: DO NOT BRING UP AN EXTRA CONTROL LAUNCH!]
        # IF TWO CONTROL LAUNCHES ARE RUNNING SIMULTANEOUSLY,
        # THERE WILL BE CORRUPTION IN THE /encoders AND /cmd_vel TOPICS
        # ============================ #
        
        # Data collection node
        data_publisher_node,
        
        # Test automation script
        test_automater
    ])
