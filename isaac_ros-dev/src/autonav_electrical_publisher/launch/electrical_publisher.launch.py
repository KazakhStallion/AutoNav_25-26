from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    # Declare launch arguments
    use_sim_time = DeclareLaunchArgument(
        'use_sim_time',
        default_value='false',
        description='Use simulation clock if true'
    )

    publish_rate = DeclareLaunchArgument(
        'publish_rate',
        default_value='10.0',
        description='Rate at which to publish electrical data (Hz)'
    )

    low_battery_threshold = DeclareLaunchArgument(
        'low_battery_threshold',
        default_value='22.0',
        description='Voltage threshold for low battery warning (V)'
    )

    critical_battery_threshold = DeclareLaunchArgument(
        'critical_battery_threshold',
        default_value='20.0',
        description='Voltage threshold for critical battery warning (V)'
    )

    # Electrical Publisher Node
    electrical_publisher_node = Node(
        package='autonav_electrical_publisher',
        executable='electrical_publisher',
        name='electrical_publisher_node',
        output='screen',
        parameters=[{
            'use_sim_time': LaunchConfiguration('use_sim_time'),
            'publish_rate': LaunchConfiguration('publish_rate'),
            'low_battery_threshold': LaunchConfiguration('low_battery_threshold'),
            'critical_battery_threshold': LaunchConfiguration('critical_battery_threshold')
        }]
    )

    return LaunchDescription([
        # Launch arguments
        use_sim_time,
        publish_rate,
        low_battery_threshold,
        critical_battery_threshold,

        # Node
        electrical_publisher_node
    ])
