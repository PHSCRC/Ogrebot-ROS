#!/usr/bin/env python
import odrive
from odrive.enums import *

import time
import math

import rospy
from geometry_msgs.msg import Twist

ROBOT_RADIUS = .12 #in meteres
WHEEL_RADIUS = .08 # in meters
ENCODER_COUNTS_PER_ROTATION = 4096

POLL_TIME=0.01
def callback(cmd_vel):
    print("linearx", cmd_vel.linear.x)
    print("angularz", cmd_vel.angular.z)
    leftWheelSpeed = cmd_vel.linear.x - (ROBOT_RADIUS*cmd_vel.angular.z) #Get linear speed of each wheel in m/s
    rightWheelSpeed = cmd_vel.linear.x + (ROBOT_RADIUS*cmd_vel.angular.z)#Get linear speed of each wheel
    leftMotorSpeed = (2*math.pi*WHEEL_RADIUS*leftWheelSpeed)#Get rotational speed of each wheel in rotations per second
    rightMotorSpeed = (2*math.pi*WHEEL_RADIUS*rightWheelSpeed)#Get rotational speed of each wheel

    my_drive.axis0.controller.vel_ramp_target = leftMotorSpeed*ENCODER_COUNTS_PER_ROTATION
    my_drive.axis1.controller.vel_ramp_target = rightMotorSpeed*ENCODER_COUNTS_PER_ROTATION


def poll(event):
    leftReading = my_drive.axis0.encoder.vel_estimate/ENCODER_COUNTS_PER_ROTATION
    rightReading = my_drive.axis1.encoder.vel_estimate/ENCODER_COUNTS_PER_ROTATION
    leftVel.publish(leftReading)
    rightVel.publish(rightReading)

def listener():
    print("looking for odrive")
    my_drive = odrive.find_any()
    my_drive.axis0.requested_state = AXIS_STATE_FULL_CALIBRATION_SEQUENCE
    my_drive.axis1.requested_state = AXIS_STATE_FULL_CALIBRATION_SEQUENCE
    while my_drive.axis0.current_state != AXIS_STATE_IDLE:
        rospy.sleep(0.1)
    my_drive.axis0.requested_state = AXIS_STATE_CLOSED_LOOP_CONTROL
    my_drive.axis0.controller.config.control_mode = CTRL_MODE_VELOCITY_CONTROL
    my_drive.axis1.requested_state = AXIS_STATE_CLOSED_LOOP_CONTROL
    my_drive.axis1.controller.config.control_mode = CTRL_MODE_VELOCITY_CONTROL

    # In ROS, nodes are uniquely named. If two nodes with the same
    # name are launched, the previous one is kicked off. The
    # anonymous=True flag means that rospy will choose a unique
    # name for our 'listener' node so that multiple listeners can
    # run simultaneously.
    rospy.init_node('listener', anonymous=False)
    rospy.Subscriber("/cmd_vel", Twist, callback)
    leftVel = rospy.Publisher('leftVel', float, queue_size=10)
    rightVel = rospy.Publisher('rightVel', float, queue_size=10)
    rospy.Timer(rospy.Duration(POLL_TIME), poll, oneshot=False)
    # spin() simply keeps python from exiting until this node is stopped
    rospy.spin()

if __name__ == '__main__':
    my_drive = None
    leftVel = None
    rightVel = None
    listener()
