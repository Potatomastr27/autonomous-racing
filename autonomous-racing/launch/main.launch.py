from launch_ros.actions import Node
 
from launch import LaunchDescription
import os
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():
    ld = LaunchDescription()

    ## Get Config
    config = os.path.join(get_package_share_directory('autonomous-racing'), 'config', 'config.yaml')

    ## Pathfinding Node
    mission_node = Node(
        package='autonomous-racing',
        executable='mission',
        name='mission',
        parameters=[config],
        output='screen'
    )    
    ld.add_action(mission_node)

    ## Controller Node
    platform_node = Node(
        package='autonomous-racing',
        executable='platform',
        name='controller',
        parameters=[config],
        output='screen'
        
    )
    ld.add_action(platform_node)

    ## Cone finder node
    cone_finder_node = Node(
        package='autonomous-racing',
        executable='coneFinder',
        name='cone_finder',
        parameters=[config],
        output='screen'
    )
    ld.add_action(cone_finder_node)

    ## Display Node
    display_node = Node(
        package='autonomous-racing',
        executable='display',
        name='display',
        output='screen'
    )
    ld.add_action(display_node)

    return ld