from launch import LaunchDescription
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():
    
    return LaunchDescription([

        Node(
            package='offline_execution',
            executable='execute_tree',
            output='screen',
        ),
    ])
