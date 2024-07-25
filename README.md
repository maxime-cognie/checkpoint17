# Checkpoint17
---

This package aims to understand the kinematic of an holonomic robot,
by applying kinematic courses onto a ROSBot XL simulated inside a Gazebo environment

## Instalation
---

#### prerequisites

 - ROS2 Humble or higher
 - Eigen 

#### Install

 1- Clone the repository inside your workspace:  
`git clone https://github.com/maxime-cognie/checkpoint17.git`  

 2- Build and setup the environment:
Navigate to the root of your workspace, then use the following command:  
```bash
colcon build
source install/setup.bash
```

## Task1    
---

```bash
cd ~/ros2_ws
source install/setup.bash
ros2 run distance_controller distance_controller
``` 

## Task2       
---

```bash
cd ~/ros2_ws
source install/setup.bash
ros2 run turn_controller turn_controller
```

## Task3       
---

```bash
cd ~/ros2_ws
source install/setup.bash
ros2 run turn_controller turn_controller 2
```

## Task4         
---

```bash
cd ~/ros2_ws
source install/setup.bash
ros2 run pid_maze_solver pid_maze_solver
```

## Task5         
---

For this task:  
1. Place the robot at the starting point like below:    
![ROSBot XL starting point](https://github.com/maxime-cognie/checkpoint17/blob/main/pid_maze_solver/resource/ROSBot_XL_start.png?raw=true)   
2. Reset the odometry by calling the **/set_pose** service: 
`ros2 service call /set_pose robot_localization/srv/SetPose "pose: {pose: {pose: {position: {x: 0.0,y: 0.0, z: 0.0}, orientation: {x: 0.0, y: 0.0, z: 0.0}}}}"` 
3. Execute the maze solver program: 
```bash
cd ~/ros2_ws
source install/setup.bash
ros2 run pid_maze_solver pid_maze_solver 2
```