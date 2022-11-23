#!/usr/bin/env python
# BEGIN ALL
from calcDestPose import look_at_target, r_from_target
from extract_by_name import extract_by_name
from getQRworldCoords import getQRworldCoords
import rospy
from geometry_msgs.msg import Twist, PoseStamped
from sensor_msgs.msg import LaserScan
from std_msgs.msg import String

import tf_conversions

import numpy as np

import time

import actionlib

from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal

from patrol import goal_pose


from gazebo_msgs.msg import ModelStates

from QR import QR

from calcTransformParams import calcTransformParams, convertLtoW


def qr_check_callback_factory(qr_array, robot_state, client):
    def qr_check_callback(msg):

        if msg.data == "":
            return

        s = msg.data
        s_split = s.split("\r\n")

        b = []

        for split in s_split:
            b.append(split.split("=")[1])

        N = int(b[4])
        X = float(b[0])
        Y = float(b[1])
        X_next = float(b[2])
        Y_next = float(b[3])
        L = b[5]

        matches = [x for x in qr_array if x.id == N]

        if not matches:
            return

        matched_qr = matches[0]
        if not matched_qr.is_initialized:
            if not robot_state.found_qr:
                robot_state.found_qr = True
                print("qr found")

            else:
                model_states = rospy.wait_for_message(
                    "/gazebo/model_states/", ModelStates, 5
                )

                robot = extract_by_name(model_states, starts_with="turtlebot3_burger")[
                    0
                ]
                robot_twist = robot[2]

                eps = 0.001

                robot_static = (
                    robot_twist.linear.x < eps
                    and robot_twist.linear.y < eps
                    and robot_twist.linear.z < eps
                    and robot_twist.angular.x < eps
                    and robot_twist.angular.y < eps
                    and robot_twist.angular.z < eps
                )
                
                if (
                    not robot_state.found_qr_static and robot_static and robot_state.allow_scan
                ) or robot_state.navigation_qr:
                    print(robot_static)

                    robot_state.found_qr_static = True

                    print("qr found static")

                    QRpose_data = rospy.wait_for_message(
                        "/visp_auto_tracker/object_position", PoseStamped, 5
                    )
                    model_states = rospy.wait_for_message(
                        "/gazebo/model_states/", ModelStates, 5
                    )

                    robot = extract_by_name(model_states, starts_with="turtlebot3_burger")[
                        0
                    ]
                    robot_pose = robot[1]
                    qr_pose = QRpose_data.pose



                    qr_world_coords = getQRworldCoords(robot_pose, qr_pose)

                    matched_qr.set_attributes(
                        X, Y, qr_world_coords[0], qr_world_coords[1], L
                    )
                    matched_qr.set_initialized()

                    print(robot_pose)
                    print(qr_pose)
                    print("local coords=", X, Y)
                    print("predicted QR=", qr_world_coords[0], qr_world_coords[1])
                    print("next QR=", X_next, Y_next)
                    print("i=", N)
                    print("l=", L)
                    
                    matches = [x for x in qr_array if x.id == (N % 5) + 1]
                    
                    matched_qr = matches[0]

                    matched_qr.set_next_qr(X_next, Y_next)

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


class RobotState:
    def __init__(self):
        self.driving_forward = True
        self.searching_qr = True
        self.found_qr = False
        self.found_qr_static = False
        self.navigation_qr = False
        self.allow_scan = False


qr_array = []
robot_state = RobotState()

rospy.init_node("wander")

client = actionlib.SimpleActionClient("move_base", MoveBaseAction)
client.wait_for_server()

for i in range(1, 6):
    qr_array.append(QR(i))

g_range_ahead = 1  # anything to start
scan_sub = rospy.Subscriber("scan", LaserScan, scan_callback)
qr_sub = rospy.Subscriber(
    "visp_auto_tracker/code_message",
    String,
    qr_check_callback_factory(qr_array, robot_state, client),
)
cmd_vel_pub = rospy.Publisher("cmd_vel", Twist, queue_size=1)

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
        robot_state.driving_forward = False
        # print "Turn"
    else:  # we're not driving_forward
        robot_state.driving_forward = True  # we're done spinning, time to go forward!
        # DRIVE
        # print "Drive"

    if not robot_state.found_qr:
        if robot_state.driving_forward:
            # print("driving forward")
            twist.linear.x = 0.4
            twist.angular.z = 0.0
        else:
            # print("rotating")
            twist.linear.x = 0.0
            twist.angular.z = 0.4

        cmd_vel_pub.publish(twist)
    else:
        twist.linear.x = 0.0
        twist.angular.z = 0.0
        cmd_vel_pub.publish(twist)

        wait_timer_start = time.time()


        rospy.sleep(1)
        robot_state.allow_scan = True
        rospy.sleep(1)
        # while (time.time() - wait_timer_start) < 2:
        #     # print("waiting")
        #     rate.sleep()

        print("Waited")            
        print(time.time() - wait_timer_start)            

        # twist.linear.x = 0.0
        # twist.angular.z = 0.4not robot_state.found_qr_static and 
        # wait_timer_start = time.time()

        # while not robot_state.found_qr_static and (time.time() - wait_timer_start) < 3:
        #     rate.sleep()

        # twist.linear.x = 0.0
        # twist.angular.z = -0.4
        # cmd_vel_pub.publish(twist)

        # wait_timer_start = time.time()

        # while not robot_state.found_qr_static and (time.time() - wait_timer_start) < 6:
        #     rate.sleep()

        robot_state.found_qr_static = False
        robot_state.found_qr = False
        robot_state.allow_scan = False


    rate.sleep()

twist = Twist()
twist.linear.x = 0.0
twist.angular.z = 0.0
cmd_vel_pub.publish(twist)

# Map QR coord fram to world coord frame
initialized_qrs = [init_qr for init_qr in qr_array if init_qr.is_initialized]

M, s, t = calcTransformParams(initialized_qrs[0], initialized_qrs[1])


def createNParr(a, b):
    return np.array([[a], [b]])

print("Mapping parameters: ")
print("M: ")
print(M)
print("s: ")
print(s)
print("t: ")
print(t)

# State: Find other QRs
robot_state.searching_qr = False
robot_state.navigation_qr = True

pose = [[-4, 0, 0], [0, 0, 0, 1]]

goal = goal_pose(pose)
client.send_goal(goal)
client.wait_for_result()

arr_len = len(qr_array)

running_index = initialized_qrs[0].id

while len(initialized_qrs) < arr_len:
    print([qr.is_initialized for qr in qr_array])
    qr = qr_array[running_index]
    if qr.is_initialized:
        running_index = (running_index + 1) % arr_len
        continue

    target_local_coords = np.array([[qr.qr_coords.x], [qr.qr_coords.y]])
    target_world_coords = convertLtoW(target_local_coords, M, s, t)

    
    model_states = rospy.wait_for_message(
        "/gazebo/model_states/", ModelStates, 5
    )

    robot = extract_by_name(model_states, starts_with="turtlebot3_burger")[
        0
    ]
    robot_position = createNParr(robot[1].position.x, robot[1].position.y)

    dest_pos = r_from_target(robot_position, target_world_coords, 1)
    dest_ori = look_at_target(dest_pos, target_world_coords)

    pose = [[robot[1].position.x, robot[1].position.y, 0], dest_ori]

    print("looking at target")

    goal = goal_pose(pose)
    client.send_goal(goal)
    client.wait_for_result()

    pose = [[dest_pos[0][0], dest_pos[1][0], 0], dest_ori]


    print("going to index")
    print(running_index+1)
    print(pose)

    goal = goal_pose(pose)
    client.send_goal(goal)
    client.wait_for_result()

    rospy.sleep(2)

    initialized_qrs = [init_qr for init_qr in qr_array if init_qr.is_initialized]
    running_index = (running_index + 1) % arr_len

twist = Twist()
twist.linear.x = 0.0
twist.angular.z = 0.0
cmd_vel_pub.publish(twist)

for qr in qr_array:
    print(qr.letter)
