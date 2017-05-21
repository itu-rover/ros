# ROS
ROS repository

To start an autonomous mission execute the following:

1. roslaunch rover_base rover_sim.launch

2. roslaunch rover_navigation move_base_mapless_demo.launch

3. rosrun rover_navigation nav2goal.py

If you need the give another waypoint please change the static waypoints located in rover_control/src/rover_velocity_controller.py

To do that while the scripts are running just simply run the following command twice:

rosrun rover_control rover_velocity_controller.py
