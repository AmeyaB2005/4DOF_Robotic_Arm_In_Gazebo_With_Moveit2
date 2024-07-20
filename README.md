# 4DOF_Robotic_Arm_In_Gazebo_With_Moveit2

# Abstract

A 4 degree-of-freedom robotic arm powered by ROS2 and controlled by Moveit2 and Gazebo.
This package contains a URDF model of a 4 DOF robotic arm equipped with a claw for gripping.



![Screenshot 2024-07-10 221005](https://github.com/AmeyaB2005/Mobile_Robot_Base_With_Arm_In_Gazebo/assets/146567207/de42720f-f575-4949-b254-de9a62432eef)


# Requirements
The ROS packages of diff_drive_controller,camera_controller ,joint_state_controller and joint_pose_trajectory_controller are required.

For example, you can install these requirements on  Ubuntu 22.04.4 LTS by the following command:<br>
   
    sudo apt install ros-humble-gazebo-ros-pkgs




# Repository architecture
## Directories
* my_robot_bringup/
  
  * launch/ : (optional) contains launch files for starting the simulation / running of the model in gazebo
  * rviz/ : (optional) contains Rviz configuration settings for displaying the robot model
  * worlds/ : (optional) contains scene/environment files for Gazebo

* my_robot_description/
  * launch/ : (optional) contains launch files for starting the simulation / running urdf model and checking the TF's
  * rviz/ : (optional) contains Rviz configuration settings for displaying the robot model
  * urdf/ : (required) contains the files that generate the robot model and provide simulated actuators and sensors
      * arm.xacro : the xacro file that generates the urdf description file of the arm
      * arm_gazebo.xacro : contains the Gazebo plugins that provide an interface to control the arm
      * camera.xacro : contains the Gazebo plugins that provide an interface to control the camera
      * common_properties.xacro : contains the common properties like different materials and macros for inertia of different geometries
      * mobile_base.xacro : the xacro file that generates the urdf description file of the mobile base of the robot
      * mobile_base_gazebo.xacro : contains the Gazebo plugins that provide an interface to control the robot wheels with differential drive
      * my_robo.urdf.xacro : includes all xacro files which are required for the robot to spawn in gazebo


# Direct usage
* Clone this repository into a ROS catkin workspace
* Build and source the workspace
* To view this robot model on RViz: $ ros2 launch my_robot_description display.launch.xml
* To launch this package in test Gazebo world and Rviz: $ ros2 launch my_robot_bringup my_robot_gazebo.launch.xml

  

# Gazebo Simulation
Control the robot inside Gazebo and view what it sees in RViz using the following launch file:

    ros2 launch my_robot_bringup my_robot_gazebo.launch.xml

This will launch the default test world **test_world.world**.

To actuate the arm use the following command:

    ros2 topic pub -1 /set_joint_trajectory trajectory_msgs/msg/JointTrajectory '{header:{frame_id: base_footprint}, joint_names: [arm_base_forearm_joint, forearm_hand_joint],points: [ {positions: {0.3, 0.4}} ]}'

Change positions as required.

![Screenshot 2024-07-10 224943](https://github.com/AmeyaB2005/Mobile_Robot_Base_With_Arm_In_Gazebo/assets/146567207/08b0a21f-38fc-453b-95ca-8d53dad455d8)


To move the mobile base use the following command:

    ros2 topic pub /cmd_vel geometry_msgs/msg/Twist "{linear: {x: 0.2}}"

Set desired velocity in the x-axis.

# RViz
View just the robot in RViz.

ros2 launch my_robot_description display.launch.xml

![Screenshot 2024-07-10 231642](https://github.com/AmeyaB2005/Mobile_Robot_Base_With_Arm_In_Gazebo/assets/146567207/75d8d885-0c5d-47d4-a9cb-0e55ea226a72)

  
# TF's


![Screenshot 2024-07-10 225158](https://github.com/AmeyaB2005/Mobile_Robot_Base_With_Arm_In_Gazebo/assets/146567207/985fa79e-bcd1-4753-867f-87c8f845c97c)

The tf system in ROS 2 keeps track of multiple coordinate frames and maintains the relationship between them in a tree structure.

# ROS2 Node Graph


![Screenshot 2024-07-10 225404](https://github.com/AmeyaB2005/Mobile_Robot_Base_With_Arm_In_Gazebo/assets/146567207/dca4b648-5268-4d6f-b1ea-b2199ebeb245)

The ROS 2 Graph​ is a network of ROS 2 elements processing data together at the same time. It encompasses all executables in nodes and the connections between them.
