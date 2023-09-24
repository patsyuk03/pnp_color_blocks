import os
from launch import LaunchDescription
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
from launch.actions import IncludeLaunchDescription, DeclareLaunchArgument
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch.conditions import IfCondition, UnlessCondition
from launch.substitutions import LaunchConfiguration



def generate_launch_description():
    ld = LaunchDescription()

    ld.add_action(DeclareLaunchArgument("add_gripper", default_value="true"))
    ld.add_action(DeclareLaunchArgument("robot_ip", default_value="xxx.xxx.x.xxx"))
    ld.add_action(DeclareLaunchArgument("use_fake_hardware", default_value="true"))

    add_gripper = LaunchConfiguration("add_gripper")
    robot_ip = LaunchConfiguration("robot_ip")
    use_fake_hardware = LaunchConfiguration("use_fake_hardware")

    # ld.add_action(IncludeLaunchDescription(
    #     PythonLaunchDescriptionSource(
    #         os.path.join(get_package_share_directory('xarm_moveit_config'), 'launch/xarm7_moveit_realmove.launch.py')
    #     ),
    #     launch_arguments={
    #         'add_gripper': add_gripper,
    #         'robot_ip': robot_ip,
    #     }.items(),
    #     condition=UnlessCondition(use_fake_hardware)
    # ))

    # ld.add_action(IncludeLaunchDescription(
    #     PythonLaunchDescriptionSource(
    #         os.path.join(get_package_share_directory('xarm_moveit_config'), 'launch/xarm7_moveit_fake.launch.py')
    #     ),
    #     launch_arguments={
    #         'add_gripper': add_gripper,
    #         'robot_ip': robot_ip,
    #     }.items(),
    #     condition=IfCondition(use_fake_hardware)
    # ))

    ld.add_action(IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(get_package_share_directory('realsense2_camera'), 'launch/rs_launch.py')
        ),
        # launch_arguments={'add_gripper': True}.items(),
    ))
    
    ld.add_action(Node(
        package="tf2_ros",
        node_executable="static_transform_publisher",
        arguments=["0.07","-0.013","0","1.571","-1.571","1.571","link_eef",'camera_link'],
        output="screen"
    ))
    ld.add_action(Node(
        package='ros2_aruco', 
        executable='aruco_node', 
        output='screen', 
        parameters=[{'camera_frame': 'camera'}]
    ))

    saved_joint_states = os.path.join(get_package_share_directory('pnp_color_blocks'), 'yaml/saved_joint_states.yaml')
    
    ld.add_action(Node(
        package="pnp_color_blocks",
        executable="pnp",
        name="saved_joint_states",
        output="screen",
        parameters=[saved_joint_states]
    ))

    return ld