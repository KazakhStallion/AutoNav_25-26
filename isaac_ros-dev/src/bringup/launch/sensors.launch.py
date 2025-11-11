import os
from ament_index_python.packages import get_package_share_directory

from launch_ros.substitutions import FindPackageShare
from launch.actions import IncludeLaunchDescription, DeclareLaunchArgument
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import PathJoinSubstitution, LaunchConfiguration
from launch import LaunchDescription



def generate_launch_description():

    zed_pkg = os.path.join(get_package_share_directory('zed_wrapper'), 'launch', 'zed_camera.launch.py')
    sick_pkg = os.path.join(get_package_share_directory('sick_scan_xd'), 'launch', 'sick_multiscan.launch.py')


    # ros2 launch sick_scan_xd sick_multiscan.launch.py hostname:=192.168.0.1 udp_receiver_ip:="192.168.0.2"

    hostname = DeclareLaunchArgument(
        'hostname',
        default_value='192.168.0.1',
        description='IP address of the SICK lidar'
    )
    udp_receiver_ip = DeclareLaunchArgument(
        'udp_receiver_ip',
        default_value='192.168.0.2',
        description='IP address of this machine'
    )
    camera_model = DeclareLaunchArgument(
        'camera_model',
        default_value='zed2i',
        description='camera_model'
    )
       
    print(f"zed: {zed_pkg}")
    print(f"sick: {sick_pkg}")

    zed = IncludeLaunchDescription(
            PythonLaunchDescriptionSource ([zed_pkg]), launch_arguments={'camera_model':LaunchConfiguration('camera_model')}.items()
    )

    sick_args = {
        'hostname':'192.168.0.1',
        'udp_receiver_ip':'192.168.0.2',
        'hostname': LaunchConfiguration('hostname'),
        'udp_receiver_ip': LaunchConfiguration('udp_receiver_ip'),
        'publish_frame_id':'lidar_footprint',
        'tf_publish_rate':'0'
    }



    sick = IncludeLaunchDescription(
            PythonLaunchDescriptionSource ([sick_pkg]), launch_arguments=sick_args.items()
    )
    return LaunchDescription([
        camera_model,
        hostname,
        udp_receiver_ip,
        zed,
        # sick
    ])

    
    
