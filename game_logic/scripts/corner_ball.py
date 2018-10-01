#!/usr/bin/env python
import rospy
from geometry_msgs.msg import Point
from geometry_msgs.msg import Twist
from math import cos, acos


x0 = 640/2
y0 = 480/2


def pos_callback(Point):
    if Point.xcoord < 20 or Point.xcoord > 620:
        vx = Point.xcoord - x0
        vy = Point.ycoord - y0
        vlength = (vx**2 + vy**2)**(0.5)
        angle = acos(vy/vlength)
        angV = (0, 0, 0)
        linV = (vlength, angle, 0)
    else:
        angV = (1, 0, 0)
        linV = (1, 0, 0)
    pub = rospy.Publisher('moving_vectors', Twist, queue_size=10)
    twistmsg = Twist(linV, angV)
    pub.publish(twistmsg)


if __name__ == "__main__":
    rospy.init_node("corner_ball")
    rospy.Subscriber("ball_coordinates", Point, pos_callback)
    rospy.spin()



