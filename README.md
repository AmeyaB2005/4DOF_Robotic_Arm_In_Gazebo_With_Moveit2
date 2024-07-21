# 4DOF_Robotic_Arm_In_Gazebo_With_Moveit2

# Abstract

A 4 degree-of-freedom robotic arm powered by ROS2 and controlled by Moveit2 and Gazebo.
This package contains a URDF model of a 4 DOF robotic arm equipped with a claw for gripping.


![Screenshot 2024-07-21 024744](https://github.com/user-attachments/assets/ee37fa76-760b-4946-a7e3-ebb4a4096bac)




# Requirements
The ROS packages of ros2_control and Moveit2 are required.

For example, you can install Moveit2 on  Ubuntu 22.04.4 LTS by the following command:<br>
   
    sudo apt install ros-humble-moveit




# Repository architecture
## Directories
* arduinobot_bringup/
  
  * launch/ : (optional) contains launch files for starting the simulation / running of the model in gazebo
  * rviz/ : (optional) contains Rviz configuration settings for displaying the robot model
  
* arduinobot_description/
  * launch/ : (optional) contains launch files for starting the simulation / running urdf model and checking the TF's
  * rviz/ : (optional) contains Rviz configuration settings for displaying the robot model
  * meshes/ : Contains all the STL files of the parts of the arm
  * models/: contains the world to be loaded in Gazebo
  * urdf/ : (required) contains the files that generate the robot model and provide simulated actuators and sensors
      * arduino_ros2_control.xacro : the xacro file that provides control to the arm
      * arduinobot_gazebo.xacro : contains the Gazebo plugins that provide an interface to control the arm
     
      * arduinobot.urdf.xacro : includes all xacro files which are required for the robotic arm to spawn in gazebo
   
* arduinobot_moveit/

   * config/ : MoveIt module for motion planning
   * launch/ : Controlling the arm through Rviz

 * arduinobot_controller/

   * arduinobot_controller/ : Controlling the arm through the MoveIt user interfaces
   * launch/ : Provides demo for the move group interface
   * config/ : MoveIt module for motion planning
  
* arduinobot_msgs/ : consists of all the actions and services required

   



# Direct usage
* Clone this repository into a ROS catkin workspace
* Build and source the workspace
* To view this robot model on RViz: $ ros2 launch arduinobot_description display.launch.xml
* To launch this package in test Gazebo world and Rviz: $ ros2 launch arduinobot_bringup simulated_robot.launch.xml

  

# Moveit

## Control real-world arm with MoveIt in RViz
Control the robot inside RViz by launching Moveit using the following launch file:

    ros2 launch arduinobot_moveit moveit.launch.py

This will launch RViz and Moveit

There should now be two interactive markers. One marker corresponding to the orange colored arm will be used to set the “Goal State” for motion planning and the other marker corresponding to a green colored arm are used to set the “Start State” for motion planning.
Now, you can start motion planning with the Arduinobot in the MoveIt RViz Plugin.

* Move the Start State to a desired location.

* Move the Goal State to another desired location.

* Make sure both states are not in collision with the robot itself.

* In the MotionPlanning window under the Planning tab, press the Plan button and then press execute.

![Screenshot 2024-07-21 114453](https://github.com/user-attachments/assets/58983036-abce-4a08-82fd-fc23bde8bc8a)

You can visually introspect trajectories point by point in RViz.

* From “Panels” menu, select “Trajectory - Trajectory Slider”. You’ll see a new Slider panel on RViz.

* Set your goal pose, then run Plan.



## Control simulated arm in Gazebo with MoveIt in RViz
View the robot in Gazebo as well as in RViz with Moveit

      ros2 launch arduinobot_bringup real_robot.launch.py

![Screenshot 2024-07-21 115341](https://github.com/user-attachments/assets/1b09541a-15b4-40b8-8f1d-ecd9fab3d8e6)

Using this you can visualize the motion.

* Plan the motion by setting the starting and the goal position in Rviz using moveit
* Then press execute
* Switch your tab to Gazebo to visualize the motion of the robotic arm as planned.





https://github.com/user-attachments/assets/e3aafbdd-e9c9-4c63-b2d7-f539164163ec








  
# TF's


![Screenshot 2024-07-21 120219](https://github.com/user-attachments/assets/bbce4122-758c-4065-9248-a1bae1add2f4)


The tf system in ROS 2 keeps track of multiple coordinate frames and maintains the relationship between them in a tree structure.

# ROS2 Node Graph


![Screenshot 2024-07-21 121401](https://github.com/user-attachments/assets/bad763ed-f798-4ea9-9784-35dafa53eb38)


The ROS 2 Graph​ is a network of ROS 2 elements processing data together at the same time. It encompasses all executables in nodes and the connections between them.
