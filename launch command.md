<!-- Different terminal -->
roslaunch final_project turtlebot3_world.launch enable_competition:=false

<!-- Different terminal -->
roslaunch final_project navigation.launch

<!-- Different terminal -->
roslaunch final_project qr_visp.launch

<!-- Different terminal -->
<!-- Remember to set local and global coastmaps aswell-->
rviz

<!-- Different terminal -->
roslaunch final_project spawn_markers.launch

roslaunch final_project spawn_obstacles.launch

rosrun final_project main.py
