
## MPC_PID_ROS2_Control

### Requirements

* Ubuntu 20.04 
* ROS2 Foxy (or newer ROS2 is also proficient) with Turtlebot3 package installed
* Ceres Solver

Note : You can also clone and build the Turtlebot3 from source.

### Install 

*  `mkdir -p ~/ros2_ws/src`
*  `cd ~/ros2_ws/src`
*   `git clone https://github.com/lenhatquang2512/MPC_PID_ROS2_Control.git`
*   `cd ..`
* `colcon build --symlink-install`
* `source install/setup.bash`

### Usage

* Run turtlebot3 gazebo emptyworld simulator:

    `ros2 launch turtlebot3_gazebo empty_world.launch.py`

* Add some obstacles by inserting some pillars 
* The goal is set (by default) at x = 5.0, y = -3.0, you can easily change the target goal later in src folder
and then build the package again.

* Open another Terminal, then give this command:

    `source install/setup.bash`

    `ros2 run hello mpc_obstacle_avoid`

* Enjoy watching the robot navigates and reached the goal and avoid all obstacles !

### Warning

* Since this is Naive MPC planner and control, setting too much obstacles are not recommended. 
It is possible, though, by tuning all the parameters. This is trial and error method, but the purpose of this package is to implement PID and MPC from scratch without using Nav2 plugin controller/planner.
* Ceres solver is a good tool to solve optimization problem, personally I think it is better than IpOpt.

