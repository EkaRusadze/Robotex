#!/usr/bin/env python
import rospy
from geometry_msgs.msg import Point
from geometry_msgs.msg import Twist
from geometry_msgs.msg import Vector3
from std_msgs.msg import Int16
import time
from math import cos, acos


x0 = 640/2
y0 = 480/2



class Game_Logic:
    def __init__(self):
        rospy.Subscriber("ball_coordinates", Point, self.ball_pos_callback)
        rospy.Subscriber("basket_coordinates", Point, self.basket_callback)
        self.pub = rospy.Publisher('moving_vectors', Twist, queue_size=10)
        self.thrower_pub = rospy.Publisher("thrower", Int16, queue_size=10)
        self.ballpointx = None
        self.ballpointy = None
        self.basketx = None
        self.baskety = None
        self.state = "idle"
        self.game_started = True
        self.timestart = None

    def ball_pos_callback(self, point):
        self.ballpointx = point.x
        self.ballpointy = point.y

    def basket_callback(self,point):
        self.basketx = point.x
        self.baskety = point.y

    def main_logic(self):
        if self.state == "idle":
            if self.game_started:
                self.state = "find_ball"
            self.stop()
        if self.state == "find_ball":
            self.rotate()
            if self.ballpointx is not None:
                self.state = "drive_to_ball"
        if self.state == "drive_to_ball":
            if self.ballpointx is None:
                self.state = "find_ball"
            else:
                self.drive_to_ball()
            if self.ballpointy > 380:
                self.state = "rotate_around_ball"
        if self.state == "rotate_around_ball":
            self.rotate_around_ball()
            if ballpointx is None:
                self.state = "find_ball"
            else:
                if self.basketx > x0 - 20 and self.basketx < x0 +20:
                    self.state = "throw_ball"
        if self.state == "throw_ball":
            if self.timestart is None:
                self.timestart = time.time()
            if time.time() - self.timestart > 1:
                self.state = "find_ball"
            else:
                self.drive_forward()
                self.throw_ball()
        if not self.game_started:
            self.state = "idle"



    def rotate(self):
        print "rotating"
        angV = Vector3(2,0,0)
        linV = Vector3(0,0,0)
        twistmsg = Twist(linV, angV)
        self.pub.publish(twistmsg)

    def stop(self):
        print "stopped"
        angV = Vector3(0,0,0)
        linV = Vector3(0,0,0)
        twistmsg = Twist(linV, angV)
        self.pub.publish(twistmsg)

    def drive_forward(self):
        print "driving forward"
        angV = Vector3(0,0,0)
        linV = Vector3(2,0,0)
        twistmsg = Twist(linV, angV)
        self.pub.publish(twistmsg)

    def drive_to_ball(self):
        print "driving to ball"
        fovangle = 75 * self.ballpointx / 640
        if fovangle <= 37.5:
            angle = 270 + fovangle * 90 / 37.5
        else:
            angle = fovangle * 90 / 37.5 - 90

        vx = self.ballpointx - x0
        vy = 480 - self.ballpointy
        vlength = (vx ** 2 + vy ** 2) ** (0.5) / 100
        angV = Vector3(0, 0, 0)
        linV = Vector3(vlength, angle, 0)
        twistmsg = Twist(linV, angV)
        self.pub.publish(twistmsg)

    def rotate_around_ball(self):
        print "rotating around ball"
        angV = Vector3(2, 0, 0)
        linV = Vector3(2, 0, 0) ## TODO find proper vectors
        twistmsg = Twist(linV, angV)
        self.pub.publish(twistmsg)


    def throw_ball(self):
        speed = self.thrower_speed()
        self.thrower_pub.publish(int(speed))

    def thrower_speed(self):
        speed = 1000 #TODO how to calculate thrower speed?
        return speed



if __name__ == "__main__":
    rospy.init_node('game_logic', anonymous=True)
    rate = rospy.Rate(60)

    game_logic = Game_Logic()

    while not rospy.is_shutdown():

        game_logic.main_logic()
        rate.sleep()



