roslaunch final_project turtlebot3_world.launch enable_competition:=false


<!-- Different terminal -->
roslaunch final_project spawn_markers.launch

roslaunch final_project spawn_obstacles.launch

<!-- Different terminal -->
rosrun gmapping slam_gmapping _xmin:=-5 _xmax:=5 _ymin:=-5 _ymax:=5

<!-- Different terminal -->
roslaunch final_project qr_visp.launch

<!-- Different terminal -->
rosrun final_project main.py
