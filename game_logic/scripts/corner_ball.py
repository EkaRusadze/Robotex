#!/usr/bin/env python
import rospy
from geometry_msgs.msg import Point
from geometry_msgs.msg import Twist
from geometry_msgs.msg import Vector3
from math import cos, acos


x0 = 640/2
y0 = 480/2

pub = rospy.Publisher('moving_vectors', Twist, queue_size=10)
def pos_callback(point):
    if point.x < x0-20 or point.x > x0+40:
        fovangle = 75 * point.x / 640
        if fovangle <= 37.5:
            angle = 270 + fovangle *90/37.5
        else:
            angle = fovangle*90/37.5-90
        vx = point.x - x0
        vy = 480 - point.y
        vlength = (vx**2 + vy**2)**(0.5)/100
        print "vlength", vlength
        #print "angle", angle
        angV = Vector3(0, 0, 0)
        linV = Vector3(vlength, angle, 0)
    else:
        print "else"
        angV = Vector3(2, 0, 0)
        linV = Vector3(0, 0, 0)

    twistmsg = Twist(linV, angV)
    pub.publish(twistmsg)

def magenta_callback(point):
    if point.x < (x0-10) or point.x>(x0+10):
        angV = Vector3(0.1, 0, 0)
        linV = Vector3(0.1, 0, 0)
        print "mag coords received"
        #magentapub = rospy.Publisher('magenta_vectors', Twist, queue_size=10)
        #twistmsg = Twist(linV, angV)
        #magentapub.publish(twistmsg)
    else:
        print "no basket"


if __name__ == "__main__":
    rospy.init_node("corner_ball")
    rospy.Subscriber("ball_coordinates", Point, pos_callback)
    rospy.Subscriber("magenta_basket_coordinates", Point, magenta_callback)
    rospy.spin()



