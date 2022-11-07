#!/usr/bin/env python
# BEGIN ALL
import rospy
from geometry_msgs.msg import Twist
from sensor_msgs.msg import LaserScan
from std_msgs.msg import String

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

        matches = [x for x in qr_array if x.id == N]

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