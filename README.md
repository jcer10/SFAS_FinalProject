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

2. In the **scripts** folder copy and make executable:
  - The **main.py** and **QR.py** from repo
  - the **keys_to_twist.py** and **key_publisher.py** from exercise 7.4

3. Create an empty folder in final_projects called **maps**.
4. *Optional* Copy the **final_project_layout.rviz** file. It is an rviz configuration file that already has all the topics to display in rviz, as is shown in the screenshot in the second exercise of the project (week 8). To open in rviz: File > Open config file.

### Create initial map (manually)
First a map has to be created of the room. To do so we need to teleoperate the robot around the room while slam mapping in rviz.

Launch the simulation with the following command:
```
roslaunch final_project turtlebot3_world.launch
```

Run the following command to start the slam mapping
```
rosrun gmapping slam_gmapping _xmin:=-5 _xmax:=5 _ymin:=-5 _ymax:=5
```

Open rviz and once inside, open configuration file (the map should then show the slam mapping of the robot in its initial position)
```
rviz
```

To teleoperate run:
```
rosrun final_project keys_to_twist.py
```
and
```
rosrun final_project key_publisher.py
```
key_publisher node is the one that actually gets key strokes, but both have to be run.

Once the you've mapped enough of the room, stop the teleoperation and save the map you made:
```
roscd final_project
mkdir maps
cd maps
rosrun map_server map_saver
```

### Run program
Once the map is created and saved, stop running the slam mapping for navigation and instead use the navigation.launch.
```
roslaunch final_project navigation.launch
```
In rviz, add the topics to the Global Coastmap and Local CoastMap.

Run the following command to see the camera output of the robot
```
roslaunch final_project qr_visp.launch
```
Then run the **main.py** script that will make the robot start wandering and reading qr codes (if it's not stupid enough to run into a corner and get stuck)
```
rosrun final_project main.py
```











