# SFAS_FinalProject

The main.py file contains the script of the main node, which subscribes to:
  - /scan topic (to read values from the laser scanner on the robot)
  - /visp_auto_tracker/code_message topic (to read message from QR code)
and publishes to:
  - /cmd_vel (to move the robot)

### to be implemented:
Subscribe to /visp_auto_tracker/object_position to obtain the QR's position in global frame.

## Instructions to run program:
