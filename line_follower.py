#!/usr/bin/env python

import signal
import rospy
import smach
import smach_ros
import math
from math import tanh
from geometry_msgs.msg import Twist
from sensor_msgs.msg import Image
import numpy as np
from sensor_msgs.msg import LaserScan
from sensor_msgs.msg import Joy

import cv2
import cv_bridge
import numpy
import time
from nav_msgs.msg import Odometry


global shutdown_requested


class Stop(smach.State):
    def __init__(self, callbacks):
        smach.State.__init__(self, outcomes=['follow_line', 'done'])
        self.callbacks = callbacks
        self.twist = Twist()
        self.cmd_vel_pub = rospy.Publisher('cmd_vel_mux/input/teleop', Twist, queue_size=1)

    def execute(self, userdata):
        global button_start
        global shutdown_requested
        start = time.time()
        print("Moving")
        while time.time() - start < 1.5:
            if shutdown_requested:
                return 'done'
            self.twist.linear.x = 0.4
            self.cmd_vel_pub.publish(self.twist)

        print("Stopping")
        self.twist.linear.x = 0
        self.cmd_vel_pub.publish(self.twist)

        start = time.time()
        while time.time() - start < 5:
            if shutdown_requested:
                return 'done'

        return 'follow_line'


class FollowLine(smach.State):

    def __init__(self, callbacks):
        smach.State.__init__(self, outcomes=['stop', 'done'])
        self.callbacks = callbacks
        self.prev_error = None
        self.Kp = 1.0 / 200.0
        self.Ki = 1.0 / 200.0
        self.Kd = 1.0 / 200.0
        self.twist = Twist()
        self.cmd_vel_pub = rospy.Publisher('cmd_vel_mux/input/teleop', Twist, queue_size=1)

    def execute(self, userdata):
        global button_start
        global shutdown_requested
        while not shutdown_requested:
            if self.callbacks.white_mask is not None and self.callbacks.red_mask is not None:
                white_mask = self.callbacks.white_mask
                red_mask = self.callbacks.red_mask

                red_pixel_count = cv2.sumElems(red_mask)[0] / 255
                if red_pixel_count > 500:
                    return 'stop'

                M = cv2.moments(white_mask)
                if M['m00'] > 0:
                    cx = int(M['m10'] / M['m00'])
                    cy = int(M['m01'] / M['m00'])
                    # BEGIN CONTROL
                    if self.prev_error is None:
                        error = cx - self.callbacks.w / 2
                        rotation = -(self.Kp * float(error))
                        self.prev_error = error
                    else:
                        error = cx - self.callbacks.w / 2
                        rotation = -(self.Kp * float(error) + self.Kd * (error - self.prev_error))
                        self.prev_error = error
                    self.twist.linear.x = 0.4
                    self.twist.angular.z = rotation
                    self.cmd_vel_pub.publish(self.twist)
                    # END CONTROL
        return 'done'


def request_shutdown(sig, frame):
    global shutdown_requested
    shutdown_requested = True


class Callbacks:
    def __init__(self):
        self.bridge = cv_bridge.CvBridge()
        self.Kp = 1.0/200.0
        self.Ki = 1.0/200.0
        self.Kd = 1.0/200.0
        self.prev_error = None
        self.past_error = []

        self.image_sub = rospy.Subscriber('camera/rgb/image_raw',
                                          Image, self.image_callback)

        self.twist = Twist()

        self.red_mask = None
        self.white_mask = None

        self.h = None
        self.w = None
        self.d = None

    def odometry_callback(self, msg):
        return

    def image_callback(self, msg):
        image = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
        hsv = cv2.cvtColor(image, cv2.COLOR_BGR2HSV)
        upper_white = numpy.array([360, 20, 255])
        lower_white = numpy.array([0,  0,  200])

        upper_red_a = numpy.array([20, 255, 255])
        lower_red_a = numpy.array([0, 100, 100])
        red_mask_a = cv2.inRange(hsv, lower_red_a, upper_red_a)

        upper_red_b = numpy.array([360, 255, 255])
        lower_red_b = numpy.array([335, 100, 100])
        red_mask_b = cv2.inRange(hsv, lower_red_b, upper_red_b)

        white_mask = cv2.inRange(hsv, lower_white, upper_white)
        red_mask = red_mask_a + red_mask_b

        self.h, self.w, self.d = image.shape
        search_top = 3*self.h/4
        search_bot = self.h
        white_mask[0:search_top, 0:self.w] = 0
        white_mask[search_bot:self.h, 0:self.w] = 0
        red_mask[0:search_top, 0:self.w] = 0
        red_mask[search_bot:self.h, 0:self.w] = 0

        self.white_mask = white_mask
        self.red_mask = red_mask

        # print(cv2.sumElems(red_mask)[0] / 255)
        cv2.imshow("white window", white_mask)
        cv2.imshow("red window", red_mask)
        cv2.waitKey(3)


def main():
    global button_start
    global shutdown_requested
    button_start = False
    shutdown_requested = False

    rospy.init_node('line_follow_bot')

    callbacks = Callbacks()
    image_sub = rospy.Subscriber('camera/rgb/image_raw', Image, callbacks.image_callback)
    odom_sub = rospy.Subscriber("odom", Odometry, callbacks.odometry_callback)

    # Create done outcome which will stop the state machine
    sm_turtle = smach.StateMachine(outcomes=['DONE'])

    with sm_turtle:
        smach.StateMachine.add('FOLLOW_LINE', FollowLine(callbacks),
                               transitions={'stop': 'STOP', 'done': 'DONE'})
        smach.StateMachine.add('STOP', Stop(callbacks),
                               transitions={'follow_line': 'FOLLOW_LINE', 'done': 'DONE'})

    # Create and start the instrospection server - needed for smach_viewer
    sis = smach_ros.IntrospectionServer('TRAVELLER_server', sm_turtle, 'STATEMACHINE')
    sis.start()

    # Start state machine and run until SIGINT received
    signal.signal(signal.SIGINT, request_shutdown)
    sm_turtle.execute()

    # Stop server
    sis.stop()


if __name__ == '__main__':
    main()