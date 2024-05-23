**Step-1**

sudo apt-get update
sudo apt-get install ros-noetic-rosserial-python ros-noetic-rosserial-arduino ros-noetic-cv-bridge python3-opencv
---------------------------------------------------------------------------------------------------
**Step-2**

mkdir -p ~/catkin_ws/src
cd ~/catkin_ws/
catkin_make
source devel/setup.bash

cd ~/catkin_ws/src
catkin_create_pkg lane_detection rospy std_msgs sensor_msgs cv_bridge
cd ~/catkin_ws
catkin_make
source devel/setup.bash
---------------------------------------------------------------------------------------------------
**Step-3**

mkdir -p ~/catkin_ws/src/lane_detection/scripts
touch ~/catkin_ws/src/lane_detection/scripts/lane_detection.py
chmod +x ~/catkin_ws/src/lane_detection/scripts/lane_detection.py
----------------------------------------------------------------------------------------------------
**Step-4**

Add the scripts
----------------------------------------------------------------------------------------------------
**Step-5**

roscore
----------------------------------------------------------------------------------------------------
**Step-6**

source ~/catkin_ws/devel/setup.bash
rosrun lane_detection lane_detection.py
