# SFAS_FinalProject

The main.py file contains the script of the main node, which subscribes to:
  - /scan topic (to read values from the laser scanner on the robot)
  - /visp_auto_tracker/code_message topic (to read message from QR code)
and publishes to:
  - /cmd_vel (to move the robot)

### to be implemented:
Subscribe to /visp_auto_tracker/object_position to obtain the QR's position in global frame.

## Instructions to run program:

### Files to add to final_project folder
1. In the **launch** folder copy from hello_ros/launch:
  - move_base.launch.xml
  - navigation.launch

Note: modify in **navigation.launch** line 25 the location of the move_base.launch.xml (final_project instead of hello_ros)

2. In the **scripts** folder copy the **main.py** and **QR.py** and make executable.

3. Create an empty folder in final_projects called **maps**

### Run program
Launch the simulation with the following command:
```
roslaunch final_project turtlebot3_world.launch
```
