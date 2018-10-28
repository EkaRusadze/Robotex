#!/usr/bin/env python


import serial
import rospy
import math
from comport_mainboard import ComportMainboard
from geometry_msgs.msg import Twist
from std_msgs.msg import String

# Constants
# Wheel angles in degrees
WHEEL_ONE_ANGLE = 270
WHEEL_TWO_ANGLE = 120
WHEEL_THREE_ANGLE = 60
WHEEL_DISTANCE_FROM_CENTER = 0.133
ROBOT_SPEED = 30
ROBOT_TURN_SPEED = 50

class Driver:
    def __init__(self):
        self.main_board = ComportMainboard()
        self.main_board.run()

        self.wheel_one_speed = 0
        self.wheel_two_speed = 0
        self.wheel_three_speed = 0
        self.throw_speed = 800

    def get_speed_for_wheel(self, wheel_angle, drive_angle,
                            robot_speed, wheel_distance_from_center,
                            robot_angular_velocity):
        move_speed = robot_speed * math.cos(math.radians(drive_angle -
                            wheel_angle))
        turn_speed = wheel_distance_from_center * robot_angular_velocity
        return move_speed + turn_speed

    def set_movement(self, linear_speed, direction_degrees, angular_speed):
        w1 = self.get_speed_for_wheel(WHEEL_ONE_ANGLE, direction_degrees,
                                      linear_speed,
                                      WHEEL_DISTANCE_FROM_CENTER,
                                      angular_speed)
        w2 = self.get_speed_for_wheel(WHEEL_TWO_ANGLE, direction_degrees,
                                      linear_speed,
                                      WHEEL_DISTANCE_FROM_CENTER,
                                      angular_speed)
        w3 = self.get_speed_for_wheel(WHEEL_THREE_ANGLE, direction_degrees,
                                      linear_speed,
                                      WHEEL_DISTANCE_FROM_CENTER,
                                      angular_speed)

        print w1, w2, w3
        self.set_wheels(int(round(w1*10)), int(round(w2*10)), int(round(w3*10)))


    def set_wheels(self, w1, w2, w3):
        rospy.loginfo("Changing wheels")
        self.wheel_one_speed = w1
        self.wheel_two_speed = w2
        self.wheel_three_speed = w3

        self.main_board.set_wheels(self.wheel_one_speed, self.wheel_two_speed, self.wheel_three_speed)

    def twist_callback(self, twist):
        print "fldkfjlsklf"
        if twist is None:
            print
            'ffs'
            return
        # print twist
        speed = twist.linear.x
        angle = twist.linear.y
        angVel = twist.angular.x
        print 'asdsfsdfsfjl'

        self.set_movement(speed, angle, angVel)

    def listener(self):
        rospy.init_node("drive")
        rospy.Subscriber("moving_vectors", Twist, self.twist_callback)
        print "smth"
        rate = rospy.Rate(5)

        while not rospy.is_shutdown():
            #rospy.Subscriber("moving_vectors", Twist, self.twist_callback)
            self.main_board.set_wheels(self.wheel_one_speed, self.wheel_two_speed, self.wheel_three_speed)
            self.main_board.set_throw(self.throw_speed)

            rate.sleep()

        self.main_board.close()

if __name__ == '__main__':
    try:

        drive = Driver()
        drive.listener()
        #rospy.Subscriber("moving_vectors", Twist, drive.twist_callback)
        #rospy.Subscriber("blue_vectors", Twist, blue_twist_callback)
        #rospy.Subscriber("magenta_vectors", Twist, magenta_twist_callback)

        #while not rospy.is_shutdown():

            #rospy.Subscriber("moving_vectors", Twist, drive.twist_callback)
            #rospy.Subscriber("moving_vectors", Twist, twist_callback)
            #twist_callback(twist_callback)
            #movement(speed, angle, angVel)
            #rate.sleep()
    except rospy.ROSInterruptException:

        pass