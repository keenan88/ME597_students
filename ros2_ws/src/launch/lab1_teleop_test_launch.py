# File: my_turtlebot3_house_launch.py

from launch import LaunchDescription
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():
    # Create a LaunchDescription instance
    ld = LaunchDescription()

    # Launch teleop_twist_keyboard
    teleop_node = Node(
        package='teleop_twist_keyboard',
        executable='teleop_twist_keyboard',
        output='screen',
        prefix = 'xterm -e'
    )
    ld.add_action(teleop_node)

    return ld
