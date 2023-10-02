import os
from launch import LaunchDescription
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
from launch.actions import IncludeLaunchDescription, DeclareLaunchArgument
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch.conditions import IfCondition, UnlessCondition
from launch.substitutions import LaunchConfiguration
from launch.launch_context import LaunchContext

def generate_launch_description():
    ld = LaunchDescription()

    ld.add_action(DeclareLaunchArgument("add_gripper", default_value="true"))
    ld.add_action(DeclareLaunchArgument("ur_type", default_value="ur5e"))
    ld.add_action(DeclareLaunchArgument("robot_ip", default_value="192.168.0.120"))
    ld.add_action(DeclareLaunchArgument("use_fake_hardware", default_value="true"))
    ld.add_action(DeclareLaunchArgument("initial_joint_controller", default_value="joint_trajectory_controller"))

    add_gripper = LaunchConfiguration("add_gripper")
    ur_type = LaunchConfiguration("ur_type")
    robot_ip = LaunchConfiguration("robot_ip")
    use_fake_hardware = LaunchConfiguration("use_fake_hardware")
    initial_joint_controller = LaunchConfiguration("initial_joint_controller")

    ld.add_action(IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(get_package_share_directory('ur_robot_driver'), 'launch/ur_control.launch.py')
        ),
        launch_arguments={
            'ur_type': ur_type,
            'robot_ip': robot_ip,
            'use_fake_hardware': use_fake_hardware,
            'launch_rviz': "false",
            'initial_joint_controller': initial_joint_controller
        }.items(),
    ))

    ld.add_action(IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(get_package_share_directory('ur_moveit_config'), 'launch/ur_moveit.launch.py')
        ),
        launch_arguments={
            'ur_type': ur_type,
            'use_fake_hardware': use_fake_hardware,
            'launch_rviz': "true",
        }.items(),
    ))

    ld.add_action(IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(get_package_share_directory('robotiq_hande_ros2_driver'), 'launch/gripper_bringup.launch.py')
        ),
        launch_arguments={
            'robot_ip': robot_ip,
        }.items(),
        condition=IfCondition(add_gripper)
    ))

    ld.add_action(Node(
        package="pnp_color_blocks",
        executable="pnp_ur",
        output="screen",
    ))

    return ld