# Run simulation

To run simulation on your local machine, follow the steps

- clone this repo to your machine and enter this repo

- source your ROS environment, if your ROS is installed in `/opt/ros` and your ros version is neotic,
  then the command is `source /opt/ros/neotic/setup.bash`

- run `catkin_make`

- run `source devel/setup.bash` according to the shell you use

- run `roslaunch gazebo_swarm_robot_tb3 gazebo_swarm_robot_5.launch` to start simulation platform

- run `rosrun gazebo_swarm_robot_tb3 gazebo_swarm_robot_control_angle` to run your angle control algorithm,

  - to run other algorithms, add target in CMakeLists.txt under `gazebo_swarm_robot_tb3` dir
  - use the command `rosrun gazebo_swarm_robot_tb3 your_target`

To stop your robot, run `rosrun gazebo_swarm_robot_tb3 gazebo_stop_robot`
