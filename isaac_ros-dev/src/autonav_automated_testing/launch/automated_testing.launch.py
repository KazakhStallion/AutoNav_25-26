import os
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource

def generate_launch_description():
    this_dir = os.path.dirname(os.path.realpath(__file__))

    test_files = [
        "t001_GPS_Cal.launch.py",
        "t002_Line_Comp.launch.py",
        "t003_Forward_Move.launch.py",
        "t004_Obj_Detect.launch.py",
        "t005_Path_Plan.launch.py",
        "t006_Simulation.launch.py",
        "t007_Speed.launch.py",
        "t008_Acceleration.launch.py",
        "t009_Rotation.launch.py",
        "t010_Steering_Drift.launch.py",
        "t011_Incline_Performance.launch.py",
    ]

    includes = []
    for f in test_files:
        path = os.path.join(this_dir, f)
        if os.path.exists(path):
            includes.append(
                IncludeLaunchDescription(PythonLaunchDescriptionSource(path))
            )

    return LaunchDescription(includes)