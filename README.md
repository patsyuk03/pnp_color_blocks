# Pick and Place color blocks

This repository is ROS 2 package for the pnp task solution for xArm7 and UR5e robots.

* [PnP xArm](#pnp-xarm): run xArm7 pnp task solution 
* [PnP UR5e](#pnp-ur5e): run UR5e ono task solution

## Prerequisites
* [Moveit 2](https://moveit.ros.org/install-moveit2/binary/)
* [Universal Robots ROS2 Driver](https://github.com/UniversalRobots/Universal_Robots_ROS2_Driver.git)
* [xArm Driver](https://github.com/xArm-Developer/xarm_ros2/tree/humble)
* [Robotiq Driver](https://github.com/patsyuk03/RobotiqHandeROS2Driver)
* [ROS2 Aruco](https://github.com/patsyuk03/ros2_aruco)
* [RealSense Driver](https://github.com/IntelRealSense/realsense-ros)


### Clone this package and build
```bash
cd ~/your_ws/src
git clone https://github.com/patsyuk03/pnp_color_blocks.git
cd ~/your_ws
colcon build
source install/setup.bash
```

## **PnP xArm** 
```bash
ros2 launch pnp_color_blocks pnp_xarm.launch.py robot_ip:=xxx.xxx.x.xxx use_fake_hardware:=false
```

## **PnP UR5e** 
### With fake hardware
```bash
ros2 launch pnp_color_blocks pnp_ur.launch.py use_fake_hardware:=true initial_joint_controller:=joint_trajectory_controller 
```
### With real hardware
```bash
ros2 launch pnp_color_blocks pnp_ur.launch.py use_fake_hardware:=false initial_joint_controller:=scaled_joint_trajectory_controller robot_ip:=xxx.xxx.x.xxx
```

## Video
[PnP xArm Hardcoded](https://drive.google.com/file/d/1uUSpr8zKzx_lnE6bKlBnnQAt5B_JegTA/view?usp=drive_link)
<br>[PnP xArm AR Detection](https://drive.google.com/file/d/1uL7H9Ht0ZtTEiZGmgDwxQWmEF36SD44k/view?usp=drive_link)
<br>[PnP UR5 Hardcoded](https://drive.google.com/file/d/1wSQwMxBCuMiS8LZaSig6x-O4G_f5scyu/view?usp=drive_link)