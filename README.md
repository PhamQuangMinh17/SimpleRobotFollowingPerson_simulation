
# TurtleBot3 Person Following Simulation

Simple simulation of a TurtleBot3 robot following a walking human.

## Description
This package demonstrates a **person-following TurtleBot3** in a simulated environment.
The simulation features a moving human model, while the robot autonomously follows the person.

This is part of a larger human-following robot project using TurtleBot3.  
The repository for the **real-world implementation** can be found [here](#).

## Video Demonstration
*https://youtu.be/vNwU35fxa-o*

## How to Use

1. Clone this repository into your ROS2 workspace and build. Below is just an example: 
   cd ~/turtlebot3_ws/src
   git clone https://github.com/PhamQuangMinh17/SimpleRobotFollowingPerson_simulation.git
   cd ~/turtlebot3_ws
   colcon build
2. Terminal 1: 
    source install/setup.bash
    export TURTLEBOT3_MODEL=waffle_pi
    ros2 launch team1_tb3_sim team1_world.launch.py
3. Terminal 2: 
    ros2 launch team1_tb3_sim team1_visualization.launch.py
4. Terminal 3: 
    source install/setup.bash
    ros2 launch team1_tb3_sim team1_follower.launch.py




