# TurtleBot Leader-Follower Project

## Required Packages
- turtlebot3
- turtlebot3_autorace_2020
- raspicam_node
- ld08_driver

## Important Info
- Make sure the IP address in ~/.bashrc under ROS_MASTER_URI matches your computer's IP
- Make sure your computer is running roscore and on the same network as turtlebot
- ROBOTIS Wiki: https://emanual.robotis.com/docs/en/platform/turtlebot3/quick-start/
- Version = ROS1 Noetic
- Linux (Ubuntu 20.04) computer required for running roscore / starting up the ROS network

## How to connect
- For first time setup, plug turtlebot into monitor to verify it's connected to network and check its IP address
- Once you know the IP address, you can SSH into the turtlebot (as long as you're on the same network) with 'ssh ubuntu@192.168.1.XXX', password 'turtlebot'
- If the Turtlebot IP address changes (if you try to SSH and it doesn't work) you have to reconnect the turtlebot to a monitor to find new IP
- Modify the turtlebot's wifi connection by editing the "50-cloud-init.yaml" file,
  - Cd /etc/netplan
  - Sudo nano 50-cloud-init.yaml
  - Sudo netplan generate, 
  - sudo netplan apply
 
  
## Launching
- "roslaunch control autonomous_driving.launch" launches everything including autonomous driving
- Specific launch files (e.g. camera, lidar, image processing) are called from the master 'autonomous_driving.launch'
- Filepath: [src/turtlebot3_autorace_2020/control/launch/autonomous_driving.launch]
- Have to manually set ROBOT_NUM environment variable, i.e. "export ROBOT_NUM=TB1"

## Editing Parameters
- Change RPi Camera/intrinsic calib through [src/turtlebot3/turtlebot3_bringup/camera_info/turtlebot3_rpicamera.yaml]
- Change camera extrinsic calib through [src/turtlebot3_autorace_2020/turtlebot3_autorace_camera/calibration/extrinsic_calibration]
- Change lane detection through [src/turtlebot3_autorace_2020/turtlebot3_autorace_detect/nodes/detect_lane]
- Change controllers through [src/seniorproject/nodes/follower_controller.py]


