import os
import yaml
from ament_index_python.packages import get_package_share_directory
from launch.actions import ExecuteProcess
from launch.substitutions import FindExecutable
from launch import LaunchDescription
from launch_ros.actions import Node

'''
This launch file is for control testing.
It starts the control node and automatically calls a service that configures the node. 
Please adjust node parameters under control/config/params.yaml

'''

def generate_launch_description():
    config = os.path.join(
        get_package_share_directory('control'),
        'config',
        'node_params.yaml'
    )
    config_config = os.path.join(
        get_package_share_directory('control'),
        'config',
        'config_params.yaml'
    ) # bruh
    with open(config_config, 'r') as f:
        all_params = yaml.safe_load(f)
        service_params = all_params.get('configure', {})  
        param_str = str(service_params).replace("'", '"')

    joy = Node(
            package='joy',
            executable='joy_node',
            name='joy_node'
        )

    control = Node(
            package='control',
            executable='control',
            name='control_node',
            parameters=[config]
        )

    configure = ExecuteProcess(
            cmd=[
                [FindExecutable(name='ros2')],
                ' service call ',
                '/configure_control ',
                'autonav_interfaces/srv/ConfigureControl ',
                f"'{param_str}'"
            ],
            shell=True,
            # delay to ensure the node is running before calling the service
            prefix='sleep 1 && '
        )

    return LaunchDescription([
        joy,
        control,
        configure
    ])

