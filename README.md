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

# Current progress

Contorl robots to form queue, offered two methods

- Based on the regidity of the graph, offered by @Smile, in `master` branch, for
  more details, check the commits message

- Based on the vector method, offered by @Sturmspatz, in
  `move_to_fix_point_queue` branch.

    - Designed by @Sturmspatz, optimized by @Smile and uploaded by @Smile.

    - Robots can move to the given positions

    - Also add some modifications to rejection effect, focus on optimizing 
    the sign of angular velocity to rotate more efficiently
