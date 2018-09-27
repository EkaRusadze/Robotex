import rospy
from geometry_msgs.msg import Point
from math import cos, acos


x0 = 640/2
y0 = 480/2

rospy.init_node("ball_pos")
rospy.Subscriber("ball_coordinates", Point, callback)

def pos_callback(Point):.
    if Point.xcoord < (x0-10) or Point.xcoord > (x0+10):
        vx = Point.xcoord - x0
        vy = Point.ycoord - y0
        vlength = sqrt(vx^2 + vy^2)
        angle = acos(vy/vlength)
        #TODO send to hardware module smth like: movement(vlength, angle, someAngVel)???????????????????????????


