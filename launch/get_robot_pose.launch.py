import os
from launch import LaunchDescription
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():
    ld = LaunchDescription()

    saved_joint_states = os.path.join(get_package_share_directory('pnp_color_blocks'), 'yaml/saved_joint_states.yaml')
    
    ld.add_action(Node(
        package="pnp_color_blocks",
        executable="get_xarm_pose",
        name="saved_joint_states",
        output="screen",
        parameters=[saved_joint_states]
    ))

    return ld