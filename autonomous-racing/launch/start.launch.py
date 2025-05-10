from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource

import os
from ament_index_python import get_package_share_directory

def generate_launch_description():
    ld = LaunchDescription()
    
    gazeboLaunchFile = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(get_package_share_directory('autonomous-racing'), 'launch/gazebo.launch.py')
        )
    )
    ld.add_action(gazeboLaunchFile)
    
    mainLaunchFile = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(get_package_share_directory('autonomous-racing'), 'launch/main.launch.py')
        )
    )
    ld.add_action(mainLaunchFile)

    
    
    return ld