# pong_bot
### Contributors: Manith Luthria, Adil Malik, Kara Lindstrom, Danny Campbell, Sarah Baird


# Overview
This robot was developed by our team as our final project for ECE 383 at Duke University. Our objective for this project was to design a robot from scratch that would be capable of shooting a ball, and therefore be able to play pong. We achieved this goal by designing the links and joints of the robot from scratch in Fusion 360 and using the fusion2urdf tool (https://github.com/syuntoku14/fusion2urdf). This tool created a URDF and launch files that allowed us to use ROS (Robot Operating System) to launch a Gazebo simulation of our robot. We then used a bash commands to manipulate the joints of the robot to move it, as well as to spawn in various items necessary for the pong game (ball, cups, table, etc). To see our robot in action, check out success.mp4.

# File Structure
code: The 'code' directory contains the Python scripts used to spawn in the game items and have the robot shoot the spawned ball.

final_assembly_backup_description: This is the (modified) directory created by the fusion2urdf tool that contains the URDF of the robot, ROS launch files, and .stl files for the robot model.

gazebo_models: This is where the models for the various game objects are saved.

# How to Use

### Requirements: ROS (developed using Noetic), a catkin workspace, Python

1. Copy this folder into your catkin workspace
2. Run 'catkin_make' and 'source devel/setup.bash' in terminal to set up dependencies
3. Run 'roslaunch final_assembly_backup_description gazebo.launch'
4. Run 'python3 final_code.py'

