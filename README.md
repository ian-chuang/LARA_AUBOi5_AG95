# LARA_Aubo_i5_PGC140

ROS packages for running Aubo i5 with DH Robotics PGC140 gripper with Moveit and Gazebo.

## Tutorial

### Dependencies
---

* Clone these into the src folder of your ROS workspace:

  ```
  cd ~/catkin_ws/src
  git clone https://github.com/ian-chuang/LARA_Aubo_i5_PGC140.git
  git clone https://github.com/ian-chuang/dh_gripper_ros.git
  git clone https://github.com/AuboRobot/aubo_robot.git -b melodic
  git clone https://github.com/roboticsgroup/roboticsgroup_gazebo_plugins.git
  rosdep install -y --from-paths . --ignore-src --rosdistro melodic
  
  cd ~/catkin_ws
  catkin build
  source devel/setup.bash
  ```
  
* Fix random bugs when building (Let me know if you have trouble)

* To run:

  ```
  roslaunch lara_moveit_config demo_gazebo.launch
  ```
