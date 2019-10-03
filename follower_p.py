#!/usr/bin/env python
# BEGIN ALL
import rospy, cv2, cv_bridge, numpy
from sensor_msgs.msg import Image
from geometry_msgs.msg import Twist

class Follower:
    def __init__(self):
        self.bridge = cv_bridge.CvBridge()
        self.Kp = 1.0/200.0
        self.Ki = 1.0/200.0
        self.Kd = 1.0/200.0
        self.prev_error = None
        self.past_error = []

        self.image_sub = rospy.Subscriber('camera/rgb/image_raw',
                                          Image, self.image_callback)
        self.cmd_vel_pub = rospy.Publisher('cmd_vel_mux/input/teleop',
                                           Twist, queue_size=1)
        self.twist = Twist()
        self.stop = False

    def image_callback(self, msg):
        image = self.bridge.imgmsg_to_cv2(msg,desired_encoding='bgr8')
        hsv = cv2.cvtColor(image, cv2.COLOR_BGR2HSV)
        upper_white = numpy.array([360, 20, 255])
        lower_white = numpy.array([0,  0,  200])
        white_mask = cv2.inRange(hsv, lower_white, upper_white)

        upper_red_a = numpy.array([20, 255, 255])
        lower_red_a = numpy.array([0, 100, 100])
        red_mask_a = cv2.inRange(hsv, lower_red_a, upper_red_a)

        upper_red_b = numpy.array([360, 255, 255])
        lower_red_b = numpy.array([335, 100, 100])
        red_mask_b = cv2.inRange(hsv, lower_red_b, upper_red_b)

        red_mask = red_mask_a + red_mask_b

        h, w, d = image.shape
        search_top = 3*h/4
        search_bot = h
        white_mask[0:search_top, 0:w] = 0
        white_mask[search_bot:h, 0:w] = 0
        red_mask[0:search_top, 0:w] = 0
        red_mask[search_bot:h, 0:w] = 0

        print(cv2.sumElems(red_mask)[0] / 255)
        cv2.imshow("Window 2", red_mask)

        red_pixel_count = cv2.sumElems(red_mask)[0]/255
        if red_pixel_count > 500 or self.stop is True:
            self.twist.linear.x = 0
            self.stop = True
            cv2.imshow("window", image)
            cv2.waitKey(3)
            return

        M = cv2.moments(white_mask)
        if M['m00'] > 0:
            cx = int(M['m10']/M['m00'])
            cy = int(M['m01']/M['m00'])
            cv2.circle(image, (cx, cy), 20, (0,0,255), -1)
            # BEGIN CONTROL
            if self.prev_error is None:
                error = cx - w/2
                self.past_error.append(error)
                # rotation = -(self.Kp*float(error) + self.Ki*(sum(self.past_error)/10))
                rotation = -(self.Kp * float(error))
                self.prev_error = error
            else:
                error = cx - w / 2
                if len(self.past_error) > 20:
                    del self.past_error[0]
                self.past_error.append(error)
                # rotation = -(self.Kp*float(error) + self.Ki*(sum(self.past_error)/10) + self.Kd*(error-self.prev_error))
                rotation = -(self.Kp*float(error) + self.Kd*(error-self.prev_error))
                self.prev_error = error
            self.twist.linear.x = 0.4
            self.twist.angular.z = rotation
            self.cmd_vel_pub.publish(self.twist)
            # END CONTROL

        cv2.imshow("window", image)
        cv2.waitKey(3)

rospy.init_node('follower')
follower = Follower()
rospy.spin()
# END ALL
