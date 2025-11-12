
import os
 
from ament_index_python.packages import get_package_share_directory
 
from launch import LaunchDescription
from launch_ros.substitutions import FindPackageShare
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import PathJoinSubstitution
import xacro
 
from launch_ros.actions import Node
 
'''
Launches entire stack.
SLAM launch can be found in SLAM package... might move it here

core launch -> TF
SLAM launch -> localization nodes and SLAM (map->odom->base_link)
sensors launch -> zed, lidar
control launch -> GPS, encoders, manual control
NAV2 launch -> pathing, costmap
'''

def generate_launch_description():
    
    slam_pkg_share = FindPackageShare('slam')
    pkg_share = FindPackageShare('bringup')
    control_share = FindPackageShare('control')
    
    slam_pkg = PathJoinSubstitution([slam_pkg_share, 'launch', 'slam.launch.py'])
    bringup_pkg = PathJoinSubstitution([pkg_share, 'launch', 'core_bringup.launch.py'])
    sensors_pkg = PathJoinSubstitution([pkg_share, 'launch', 'sensors.launch.py'])
    control_pkg = PathJoinSubstitution([control_share, 'launch', 'control.launch.py'])
    
    bowser = IncludeLaunchDescription( PythonLaunchDescriptionSource (bringup_pkg) )
    slam = IncludeLaunchDescription( PythonLaunchDescriptionSource (slam_pkg) )
    sensors = IncludeLaunchDescription( PythonLaunchDescriptionSource (sensors_pkg) )
    control = IncludeLaunchDescription( PythonLaunchDescriptionSource (control_pkg) )
    
    
    
    return LaunchDescription([
        bowser,
        sensors,
        slam,
        control
    ])



    
