#!/usr/bin/env python
# BEGIN ALL
from extract_by_name import extract_by_name
from getQRworldCoords import getQRworldCoords
import rospy
from geometry_msgs.msg import Twist, PoseStamped
from sensor_msgs.msg import LaserScan
from std_msgs.msg import String

import tf_conversions

import numpy as np


from gazebo_msgs.msg import ModelStates

from QR import QR

from calcTransformParams import calcTransformParams, convertLtoW


def qr_check_callback_factory(qr_array):
    def qr_check_callback(msg):

        if msg.data == "":
            return

        s = msg.data
        s_split = s.split("\r\n")

        # print("here")
        # print(s_split)

        b = []

        for split in s_split:
            b.append(split.split("=")[1])

        N = int(b[4])
        X = float(b[0])
        Y = float(b[1])
        X_next = float(b[2])
        Y_next = float(b[3])
        L = b[5]

        model_states = rospy.wait_for_message("/gazebo/model_states/", ModelStates, 5)
        QRpose_data = rospy.wait_for_message(
            "/visp_auto_tracker/object_position", PoseStamped, 5
        )
        # print(QRpose_data)
        # print(dir(QRpose_data))

        robot = extract_by_name(model_states, starts_with="turtlebot3_burger")[0]
        # print(robot)
        robot_pose = robot[1]
        qr_pose = QRpose_data.pose

        robot_rot = tf_conversions.transformations.quaternion_matrix(
            [
                robot_pose.orientation.x,
                robot_pose.orientation.y,
                robot_pose.orientation.z,
                robot_pose.orientation.w,
            ]
        )
        # print("robot=", robot)
        # print("qr_pose=", qr_pose)

        qr_world_coords = getQRworldCoords(robot_pose, qr_pose)

        matches = [x for x in qr_array if x.id == N]

        if not matches:
            return

        matched_qr = matches[0]

        if not matched_qr.is_initialized:
            print("qr found")

        matched_qr.set_attributes(X, Y, qr_world_coords[0], qr_world_coords[1], L)
        matched_qr.set_initialized()

        # print("predicted QR=", qr_world_coords[0], qr_world_coords[1])

        matches = [x for x in qr_array if x.id == (N % 5) + 1]

        matched_qr = matches[0]

        matched_qr.set_attributes(X, Y, 0, 0, L)

        # print(matched_qr.letter)

    return qr_check_callback


def scan_callback(msg):
    global g_range_ahead
    # print(g_range_ahead)
    tmp = [msg.ranges[0]]
    for i in range(1, 21):
        tmp.append(msg.ranges[i])
    for i in range(len(msg.ranges) - 21, len(msg.ranges)):
        tmp.append(msg.ranges[i])
    g_range_ahead = min(tmp)


qr_array = []

for i in range(1, 6):
    qr_array.append(QR(i))

g_range_ahead = 1  # anything to start
scan_sub = rospy.Subscriber("scan", LaserScan, scan_callback)
qr_sub = rospy.Subscriber(
    "visp_auto_tracker/code_message", String, qr_check_callback_factory(qr_array)
)
cmd_vel_pub = rospy.Publisher("cmd_vel", Twist, queue_size=1)
rospy.init_node("wander")
state_change_time = rospy.Time.now() + rospy.Duration(1)
driving_forward = True
rate = rospy.Rate(60)


# State: Wander
while not rospy.is_shutdown():
    twist = Twist()

    count = len([init_qr for init_qr in qr_array if init_qr.is_initialized])
    if count == 2:

        twist.linear.x = 0.0
        twist.angular.z = 0.0
        cmd_vel_pub.publish(twist)
        break

    # print(count)
    # print(driving_forward)
    if g_range_ahead < 0.8:
        # TURN
        driving_forward = False
        # print "Turn"

    else:  # we're not driving_forward
        driving_forward = True  # we're done spinning, time to go forward!
        # DRIVE
        # print "Drive"

    if driving_forward:
        # print("driving forward")
        twist.linear.x = 0.4
        twist.angular.z = 0.0
    else:
        # print("not driving forward")
        twist.linear.x = 0.0
        twist.angular.z = 0.4
        cmd_vel_pub.publish(twist)

    rate.sleep()

# Map QR coord fram to world coord frame
initialised_qrs = [init_qr for init_qr in qr_array if init_qr.is_initialized]

for qr in initialised_qrs:
    print("local")
    print(qr.qr_coords.x)
    print(qr.qr_coords.y)
    print("world")
    print(qr.world_coords.x)
    print(qr.world_coords.y)

M, s, t = calcTransformParams(*initialised_qrs)


def createNParr(a, b):
    return np.array([[a], [b]])


qrs = [
    [3, 1.2],
    [2.67, 3.23],
    [0.1, 3.5],
    [-3.1, 2],
    [-3.08, 0],
]

qrs = [createNParr(*qr) for qr in qrs]

# print(M)
# print(t)
qrs_W = [convertLtoW(qr, M, s, t) for qr in qrs]

for i, qr in enumerate(qrs_W):
    print("marker {}:".format(i))
    print(qr)

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
