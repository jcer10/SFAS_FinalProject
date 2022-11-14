#!/usr/bin/env python
# BEGIN ALL
from extract_by_name import extract_by_name
import rospy
from geometry_msgs.msg import Twist
from sensor_msgs.msg import LaserScan
from std_msgs.msg import String

import tf_conversions

import numpy as np


from gazebo_msgs.msg import ModelStates

from QR import QR


def qr_check_callback_factory(qr_array):


    def qr_check_callback(msg):


        if msg.data == "": return

        s = msg.data
        s_split = s.split("\r\n")

        print("here")
        print(s_split)

        b = []

        for split in s_split:
            b.append(split.split("=")[1])

        N = float(b[4])
        X = float(b[0])
        Y = float(b[1])
        X_next = float(b[2])
        Y_next = float(b[3])
        L = b[5] 

        model_states = rospy.wait_for_message('/gazebo/model_states/', ModelStates, 5)
        scan_data = rospy.wait_for_message('scan', LaserScan, 5)

        tmp=[msg.ranges[0]]
        for i in range(1, 11):
          tmp.append(scan_data.ranges[i])
        for i in range(len(scan_data.ranges)-11, len(msg.ranges)):
          tmp.append(scan_data.ranges[i])
        dist = min(tmp)

        robot = extract_by_name(
            model_states, starts_with="turtlebot3_burger")[0]

        robot_pose = robot.position
        robot_rot = tf_conversions.transformations.quaternion_matrix(robot.orientation.x, robot.orientation.y, robot.orientation.z, robot.orientation.w)

        qr_world_coords = np.array([[robot_pose.x], [robot_pose.y], [robot_pose.z]]) + robot_rot[:3, :3] * np.array([dist], [0], [0])

        matches = [x for x in qr_array if x.id == N]

        matched_qr = matches[0]

        matched_qr.set_attributes(X, Y, qr_world_coords[0], qr_world_coords[1], L)
        matched_qr.set_initialized()

        matches = [x for x in qr_array if x.id == N+1]

        matched_qr = matches[0]

        matched_qr.set_attributes(X, Y, 0, 0, L)



        print(matched_qr.letter)
        
    return(qr_check_callback)








def scan_callback(msg):
  global g_range_ahead
  tmp=[msg.ranges[0]]
  for i in range(1,21):
    tmp.append(msg.ranges[i])
  for i in range(len(msg.ranges)-21,len(msg.ranges)):
    tmp.append(msg.ranges[i])
  g_range_ahead = min(tmp)


qr_array = []

for i in range(1, 6):
    qr_array.append(QR(i))
 
g_range_ahead = 1 # anything to start
scan_sub = rospy.Subscriber('scan', LaserScan, scan_callback)
qr_sub = rospy.Subscriber('visp_auto_tracker/code_message', String, qr_check_callback_factory(qr_array))
cmd_vel_pub = rospy.Publisher('cmd_vel', Twist, queue_size=1)
rospy.init_node('wander')
state_change_time = rospy.Time.now() + rospy.Duration(1)
driving_forward = True
rate = rospy.Rate(60)


# State: Wander
while not rospy.is_shutdown():


  #print g_range_ahead
  if g_range_ahead < 0.8:
    # TURN
    driving_forward = False
    #print "Turn"
   
  else: # we're not driving_forward
    driving_forward = True # we're done spinning, time to go forward!
    #DRIVE
    #print "Drive"
   
  twist = Twist()
  if driving_forward:
    twist.linear.x = 0.4
    twist.angular.z = 0.0
  else:
    twist.linear.x = 0.0
    twist.angular.z = 0.4
  cmd_vel_pub.publish(twist)
 
  rate.sleep()
# END ALL

# # State: Find other QRs
# while not rospy.is_shutdown():


#   #print g_range_ahead
#   if g_range_ahead < 0.8:
#     # TURN
#     driving_forward = False
#     #print "Turn"
   
#   else: # we're not driving_forward
#     driving_forward = True # we're done spinning, time to go forward!
#     #DRIVE
#     #print "Drive"
   
#   twist = Twist()
#   if driving_forward:
#     twist.linear.x = 0.4
#     twist.angular.z = 0.0
#   else:
#     twist.linear.x = 0.0
#     twist.angular.z = 0.4
#   cmd_vel_pub.publish(twist)
 
#   rate.sleep()
# # END ALL