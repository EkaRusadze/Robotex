#!/usr/bin/env python
import rospy
from geometry_msgs.msg import Point
from geometry_msgs.msg import Twist
from geometry_msgs.msg import Vector3
from math import cos, acos


x0 = 640/2
y0 = 480/2


def pos_callback(Point):
    #print Point
    if Point.x < (x0-10) or Point.x > (x0+10):
        angV = Vector3(0.1, 0, 0)
        linV = Vector3(0.1, 0, 0)
    else:
        vx = Point.x - x0
        vy = Point.y - y0
        vlength = (vx**2 + vy**2)**(0.5)
        angle = acos(vy/vlength)
        angV = Vector3(0, 0, 0)
        linV = Vector3(vlength, 0, 0)
        #distance = 480 - Point.y
         #   if distance > 400:

    pub = rospy.Publisher('moving_vectors', Twist, queue_size=10)
    twistmsg = Twist(linV, angV)
    pub.publish(twistmsg)
    print twistmsg




if __name__ == "__main__":
    rospy.init_node("center_ball")
    rospy.Subscriber("ball_coordinates", Point, pos_callback)
    rospy.spin()
