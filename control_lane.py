#!/usr/bin/env python
# -*- coding: utf-8 -*-

################################################################################
# Copyright 2018 ROBOTIS CO., LTD.
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.
################################################################################

# Author: Leon Jung, Gilbert
 
import rospy
import numpy as np
from std_msgs.msg import Float64, Int8, UInt8
from geometry_msgs.msg import Twist
import time

lastError = 0
MAX_VEL = 0.25 #0.12

path=["right", "right", "right", "right", "right", "right","right"]
path_node=1
follow=path[path_node]

left_data=Float64()
right_data=Float64()
previous_center=Float64()
turning_data=Int8()
left_data.data=300.0
right_data.data=300.0
previous_center.data=300.0
turning_data.data=-1

Stop=False
check=False

turning=False
def cbFollowLane(desired_center):
    global lastError, follow, path_node, previous_center, turning
    global average
    global MAX_VEL
    global Stop
    # print "center_data: ", desired_center.data
    if follow=="center":
        center = desired_center.data
        # if turning_data.data!=0:
        #     print "crossroad"
        #     turning=True
        #     time.sleep(9)
                
        # elif turning:
        #     print "passed"
        #     turning=False
        #     path_node+=1
        #     follow=path[path_node]
                
    elif follow=="left":
        center = left_data.data
        # if turning_data.data==1:
        #     turning=True
        #     print "turning_left"
        # elif turning:
        #     print follow, turning, turning_data.data
        #     print "turned"
        #     turning=False
        #     path_node+=1
        #     follow=path[path_node]
        #     time.sleep(7)
                
    elif follow=="right":
        center = right_data.data
        
        # if turning_data.data==2:
        #     turning=True
        #     print "turning_right"
        # elif turning:
        #     print "turned"
        #     turning=False
        #     path_node+=1
        #     follow=path[path_node]
        #     time.sleep(7)
            
    else:
        center=previous_center

    previous_center=center
    try:
        error = int(center) - 300
    except Exception as e:
        print center
        print e
        return

    if Stop:
        fnShutDown()
        return
    Kp = 0.0025 #(math.pi/2)*(1/infinity_distance) #now 628
    Kd = 0.007

    angular_z = Kp*error + Kd*(error - lastError)
    lastError = error
    twist = Twist()
    twist.linear.x = min(MAX_VEL*((1 - abs(error)/300)**2.2), 0.2)
    twist.linear.y = 0
    twist.linear.z = 0
    twist.angular.x = 0
    twist.angular.y = 0
    twist.angular.z = -max(angular_z, -2.0) if angular_z < 0 else -min(angular_z, 2.0)
    pub_cmd_vel.publish(twist)

def left_lane(left):
    global left_data
    left_data=left
    # print "left_data:", left_data.data
def right_lane(right):
    global right_data
    right_data=right
    # print "right_data:", right_data.data
def turning_side(data):
    global turning_data
    turning_data=data

def fnShutDown():
    time.sleep(1)
    # rospy.loginfo("Shutting down. cmd_vel will be 0")

    twist = Twist()
    twist.linear.x = 0
    twist.linear.y = 0
    twist.linear.z = 0
    twist.angular.x = 0
    twist.angular.y = 0
    twist.angular.z = 0
    pub_cmd_vel.publish(twist) 

def sign(value):
    global Stop,check
    print value
    if value.data==3 and not check:
        Stop=True
        check=True
        print value,Stop,time.time()
        time.sleep(30)
        Stop=False
        print Stop,time.time()
        

if __name__ == '__main__':
    rospy.on_shutdown(fnShutDown)
    rospy.init_node('control_lane')
    sub_lane = rospy.Subscriber('/control/lane', Float64, cbFollowLane, queue_size = 1)
    left = rospy.Subscriber('/control/lane/left', Float64, left_lane, queue_size = 1)
    right = rospy.Subscriber('/control/lane/right', Float64, right_lane, queue_size = 1)
    turning_sub = rospy.Subscriber('/control/turning', Int8, turning_side, queue_size = 1)
    sign = rospy.Subscriber("/detect/sign_number", UInt8, sign, queue_size=1)
    pub_cmd_vel = rospy.Publisher('cmd_vel', Twist, queue_size = 1)
    rospy.spin()

