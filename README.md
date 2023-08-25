# LARA_AUBOi5_AG95

Welcome to the **LARA_AUBOi5_AG95** repository! This repository contains ROS packages designed for running the Aubo i5 robot with the DH Robotics AG95 gripper. The primary focus of this repository is to provide tools and resources for experimenting with pick and place tasks using the Moveit motion planning framework and simulating in Gazebo. The project was developed within the context of the LARA lab.

## Features

- Integration of Aubo i5 robot with DH Robotics AG95 gripper.
- Simulation environment setup in Gazebo for pick and place tasks.
- Implementation of Moveit for motion planning and manipulation.
- Teleoperation capabilities using a Meta Quest 2 VR Hand Controller.
- Teleoperation support via a joystick.

## Note

Please note that this repository is not actively maintained. It was created during a learning phase while exploring the capabilities of ROS (Robot Operating System) and experimenting with robotics concepts. As a result, the code and documentation provided here might be incomplete or outdated. While the content remains available for reference and inspiration, there are no guarantees of ongoing updates or support.

## Videos

Check out some cool videos showcasing the environment in action:

![Aubo Gazebo Pick and Place](media/aubo_gazebo.gif)

![Aubo Meta Quest 2 Hand Controller Teleoperation](media/aubo_vr.gif)

## Tutorial

To get started with using the **LARA_AUBOi5_AG95** repository and setting up the required environment, follow these steps:

1. **Update System and Install Dependencies**:

    ```
    # Update
    rosdep update
    sudo apt update
    sudo apt dist-upgrade
  
    # Dependencies to install MoveIt from source
    sudo apt install python3-wstool python3-catkin-tools clang-format-10 python3-rosdep
    ```

2. **Create Workspace and Clone Repositories**:

    ```
    # Make workspace
    mkdir -p ~/catkin_ws/src
  
    # Clone packages
    cd ~/catkin_ws/src
    git clone https://github.com/ian-chuang/LARA_AUBOi5_AG95.git 
    git clone https://github.com/ian-chuang/dh_gripper_ros.git # AG95 gripper description and drivers
    git clone https://github.com/ian-chuang/aubo_robot.git -b melodic # Aubo i5 description and drivers
    git clone https://github.com/ian-chuang/oculus_reader.git # Reading Meta Quest 2 Controllers
    ```

3. **Clone Gazebo Plugins**:

    ```
    # Clone gazebo plugins
    cd ~/catkin_ws/src
    mkdir gazebo_pkgs
    cd gazebo_pkgs
    git clone https://github.com/roboticsgroup/roboticsgroup_upatras_gazebo_plugins.git # mimic joint plugin
    git clone https://github.com/pal-robotics/realsense_gazebo_plugin.git -b melodic-devel # realsense simulation plugin
    git clone https://github.com/ian-chuang/gazebo_gripper_action_controller.git # modified gripper action controller for gazebo
    ```

4. **Install MoveIt from Source**:

    ```
    # Install MoveIt from source
    cd ~/catkin_ws/src
    mkdir moveit
    wstool init moveit
    wstool merge -t moveit https://raw.githubusercontent.com/ros-planning/moveit/melodic-devel/moveit.rosinstall
    wstool update -t moveit
    ```

5. **Install ROS Package Dependencies**:

    ```
    # Install ROS package dependencies
    cd ~/catkin_ws/src
    rosdep install -y --from-paths . --ignore-src --rosdistro melodic
    ```

6. **Install Other Dependencies LARA Uses**:

    ```
    # Install other dependencies LARA uses
    sudo apt-get install ros-melodic-realsense2-camera # realsense drivers
    ```

7. **Add Aubo Dependencies**:

    ```
    # Add aubo dependencies 
    sudo apt-get install gdebi
    wget http://archive.ubuntu.com/ubuntu/pool/main/p/protobuf/libprotobuf9v5_2.6.1-1.3_amd64.deb 
    sudo gdebi ./libprotobuf9v5_2.6.1-1.3_amd64.deb
    rm ./libprotobuf9v5_2.6.1-1.3_amd64.deb
    cd ~/catkin_ws/src
    sudo ln -s aubo_robot/aubo_driver/lib/lib64/config/libconfig.so.11 /usr/lib/libconfig.so.11
    sudo ln -s aubo_robot/aubo_driver/lib/lib64/log4cplus/liblog4cplus-1.2.so.5.1.4 /usr/lib/liblog4cplus-1.2.so.5.1.4
    sudo ldconfig
    ```

8. **Build Workspace**:

    ```
    # Build
    cd ~/catkin_ws
    catkin build
    source devel/setup.bash
    ```
    
## Getting Started and Running the Simulation

Before you start, make sure you have completed the setup steps mentioned in the previous sections.

* **Setting Up Meta Quest 2 Teleop**:
  To enable teleoperation using the Meta Quest 2 VR Hand Controller, follow the instructions provided in the [oculus_reader README](https://github.com/ian-chuang/oculus_reader/blob/main/README.md).

* **Running Simulation and MoveIt**:
  To run the simulation and MoveIt, execute the following commands:

  ```bash
  # Launch simulation and MoveIt
  roslaunch lara_moveit_config demo_gazebo.launch cameras:=true
  
  # Spawn objects in the simulation
  roslaunch lara_description spawn_parts.launch
  
  # Run the pick routine
  rosrun lara_manipulation pick_and_place.py
  ```
  
* **Realtime Servoing with Meta Quest 2 Teleop**:
  If you want to experience realtime servoing using the Meta Quest 2 VR Hand Controller, follow these steps:

  ```bash
  # Ensure that simulation and MoveIt are running
  
  # Switch from trajectory controller to position controller
  rosservice call /controller_manager/switch_controller "start_controllers:
  - 'arm_position_controller'
  stop_controllers:
  - 'arm_trajectory_controller'
  strictness: 2"
  
  # Run the realtime servoing server
  roslaunch lara_moveit_servo pose_tracker.launch
  
  # Run the Meta Quest teleop
  rosrun oculus_reader pose_teleop.py
  ```

## Changes I Did to Get It Working

In the process of working with this repository, I made several adjustments to ensure its functionality. Here's a breakdown of the changes I made:

#### lara_description:
* **aubo_i5_macro.xacro**: Created a new URDF macro specifically for the Aubo i5 robot. This macro includes added joint transmissions to facilitate proper joint movement.
* **pgc140_macro.xacro**: Developed a URDF macro for the PGC140 Gripper, incorporating joint transmissions and a mimic joint gazebo plugin to accurately simulate the gripper's behavior.
* **aubo_i5_pgc140.urdf.xacro**: This URDF file combines the Aubo i5 arm and the PGC140 Gripper. It attaches the gripper to the arm, fixes them to the world, and integrates the necessary gazebo ROS control plugin for simulation.
* **display.launch**: Created a launch file, allowing the URDF model to be visualized in RViz.

#### lara_moveit_config (auto-generated by moveit setup assistant):
* **gazebo_controllers.yaml**: Generated a configuration file that defines the arm and gripper controllers. This file also includes PID values for the mimic joint gazebo plugin, ensuring accurate joint control during simulation.
* **ros_controllers.yaml**: Modified the ROS controllers configuration file to enable proper interfacing with the gazebo controllers.
* **gazebo.launch**: Adjusted the gazebo.launch file to correctly load the xacro macro as a URDF. Additionally, the modifications allow the controllers defined in gazebo_controllers.yaml to be spawned in the simulation.

**Disclaimer:** The information provided here is based on the changes I applied to the repository according to the description provided. Actual changes and code adjustments may vary depending on the specifics of your ROS setup and the codebase at the time of your interaction with it.
