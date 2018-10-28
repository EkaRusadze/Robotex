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
        vx = point.x - x0
        vy = point.y - y0
        vlength = (vx**2 + vy**2)**(0.5)
        angle = acos(vy/vlength)
        angV = Vector3(0, 0, 0)
        linV = Vector3(3, angle, 0)
    else:
        angV = Vector3(2, 0, 0)
        linV = Vector3(0, 0, 0)

    twistmsg = Twist(linV, angV)
    pub.publish(twistmsg)


if __name__ == "__main__":
    rospy.init_node("corner_ball")
    rospy.Subscriber("ball_coordinates", Point, pos_callback)
    rospy.spin()



