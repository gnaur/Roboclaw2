import launch
import os

from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch_ros.actions import Node



def generate_launch_description():

    pkg_path = get_package_share_directory('roboclaw2')

    config = os.path.join(pkg_path,'config','params.yaml')
    
    
    driver_node=Node(
        package='roboclaw2',
        executable='roboclaw2_node',
        output='screen',
        name='roboclaw',
        parameters=[config]
    )

    return LaunchDescription([
        driver_node
    ])


