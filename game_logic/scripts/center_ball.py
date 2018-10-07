#!/usr/bin/env python
import rospy
from geometry_msgs.msg import Point
from geometry_msgs.msg import Twist
from geometry_msgs.msg import Vector3
from math import cos, acos, atan2


x0 = 640/2
y0 = 480/2

pub = rospy.Publisher('moving_vectors', Twist, queue_size=10)
def pos_callback(point):
    #print Point
    if point.x < (x0-20) or point.x > (x0+20):
        #print "if", point
        angV = Vector3(0.1, 0, 0)
        linV = Vector3(0.1, 0, 0)
    else:
        #print "else", point
        vx = point.x - x0
        vy = point.y - y0
        vlength = (vx**2 + vy**2)**(0.5)
        angle = atan2(vy, vx)
        angV = Vector3(0, 0, 0)
        linV = Vector3(0.1, 0, 0)
        distance = 480 - point.y

    twistmsg = Twist(linV, angV)
    pub.publish(twistmsg)
    print twistmsg

# def blue_callback(point):
#     if point.x < (x0-10) or point.x>(x0+10):
#         angV = Vector3(0.1, 0, 0)
#         linV = Vector3(0.1, 0, 0)
#         bluepub = rospy.Publisher('blue_vectors', Twist, queue_size=10)
#         twistmsg = Twist(linV, angV)
#         bluepub.publish(twistmsg)
#     else:
#         print "No blue basket"


# def magenta_callback(point):
#     if point.x < (x0-10) or point.x>(x0+10):
#         angV = Vector3(0.1, 0, 0)
#         linV = Vector3(0.1, 0, 0)
#         magentapub = rospy.Publisher('magenta_vectors', Twist, queue_size=10)
#         twistmsg = Twist(linV, angV)
#         magentapub.publish(twistmsg)
#     else:
#         print "No magenta basket"
# # TODO which basket??


if __name__ == "__main__":
    rospy.init_node("center_ball")
    rospy.Subscriber("ball_coordinates", Point, pos_callback)

    #rospy.Subscriber("blue_basket_coordinates", Point, blue_callback)
    #rospy.Subscriber("magenta_basket_coordinates", Point, magenta_callback)
    rospy.spin()
