import rospy
from geometry_msgs.msg import Point
from math import cos, acos


x0 = 640/2
y0 = 480/2


def pos_callback(Point):.
    if Point.xcoord < (x0-10) or Point.xcoord > (x0+10):
        angV = (1, 0, 0)
        linV = (1, 0, 0)
    else:
        vx = Point.xcoord - x0
        vy = Point.ycoord - y0
        vlength = sqrt(vx^2 + vy^2)
        angle = acos(vy/vlength)
        angV = (0, 0, 0)
        linV = (vlength, 0, 0)
    pub = rospy.Publisher('moving_vectors', Twist, queue_size=10)
    twistmsg = Twist(Vector3(linV), Vector3(angV)) #first vector is for linear movement, second is for angular
    pub.publish(twistmsg)


if __name__ = "__main__":
    rospy.init_node("center_ball")
    rospy.Subscriber("ball_coordinates", Point, callback)
    pos_callback(Point)



