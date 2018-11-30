#!/usr/bin/env python
import rospy
from geometry_msgs.msg import Point
from geometry_msgs.msg import Twist
from geometry_msgs.msg import Vector3
from std_msgs.msg import Int16
import time
from math import cos, acos

width = 640
height = 480
x0 = width/2
y0 = height/2
offset = 50




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
        print "ball pointx", point.x
        if point.x == 1000:
            self.ballpointx = None
            self.ballpointy = None
        else:
            self.ballpointx = point.x -offset
            self.ballpointy = point.y


    def basket_callback(self,point):
        print "basket pointx", point.x
        if point.x == 1000:
            self.basketx = None
            self.baskety = None
        else:
            self.basketx = point.x - offset
            self.baskety = point.y

    def main_logic(self):
        #self.test_speed()
        #return

        if self.state == "idle":
            if self.game_started:
                self.state = "find_ball"
            self.stop()
        if self.state == "find_ball":
            self.rotate()
            if self.ballpointx is not None:
                self.state = "drive_to_ball"
        #if self.state == "center_ball":
            #self.rotate()
            #if self.ballpointx > x0 -20  and self.ballpointx < x0+20 :
             #   self.state = "drive_to_ball"
        if self.state == "drive_to_ball":
            if self.ballpointx is None:
                self.state = "find_ball"
            #elif self.ballpointx < x0 -20 and self.ballpointx > x0 +20:
             #   self.state = "center_ball"
            else:
                self.drive_to_ball()
            if self.ballpointy > 415 and self.ballpointx > x0 - 40 and self.ballpointx < x0 +40:
                #self.game_started = False
                #self.state = "idle"
                self.state = "rotate_around_ball"
        if self.state == "rotate_around_ball":
            if self.ballpointx is None:
                self.state = "find_ball"
            if self.basketx is None:
                self.rotate_around_ball()
            else:
                if self.basketx < x0-20:
                    self.rotate_around_ball()
                elif self.basketx > x0 +20:
                    self.rotate_around_ball()
                else:
                    self.state = "throw_ball"
        if self.state == "throw_ball":
            if self.timestart is None:
                self.timestart = time.time()
            if time.time() - self.timestart > 1:
                self.state = "find_ball"
            else:
                if self.ballpointx is None:
                    self.state = "find_ball"
                if basketx is None:
                    self.state = "rotate_around_ball"
                else:
                    self.drive_forward()
                    print "after drive forward"
                    #time.sleep() ??
                    self.throw_ball()
                    self.timestart = None
                    print "after"
        if not self.game_started:
            self.state = "idle"

    def test_speed(self):
        print "test_speed"
        angV = Vector3(10, 0, 0)
        linV = Vector3(2, 90, 0)
        twistmsg = Twist(linV, angV)
        self.pub.publish(twistmsg)

    def rotate(self):
        print "rotating"
        angV = Vector3(20,0,0)
        linV = Vector3(0,0,0)
        twistmsg = Twist(linV, angV)
        self.pub.publish(twistmsg)

    def stop(self):
        print "stopped"
        angV = Vector3(0,0,0)
        linV = Vector3(0,0,0)
        twistmsg = Twist(linV, angV)
        self.pub.publish(twistmsg)

    def drive_to_ball(self):
        print "coordinates:", self.ballpointx, self.ballpointy
        print "driving to ball"
        deltax = x0 - self.ballpointx
        normdelta = -deltax/width
        maxspeed = 50
        angVel = maxspeed*normdelta
        maxfwdspeed =15
        dst = height - self.ballpointy * 0.7
        normdst = dst/height
        fwdspeed = maxfwdspeed*(normdst**2)
        if abs(normdelta) < 0.02:
            angVel = 0
        angV = Vector3(angVel,0,0)
        linV = Vector3(fwdspeed,0,0)
        twistmsg = Twist(linV, angV)
        self.pub.publish(twistmsg)

    def drive_forward(self):
        print "driving forward"
        angV = Vector3(0,0,0)
        linV = Vector3(2,0,0)
        twistmsg = Twist(linV, angV)
        self.pub.publish(twistmsg)

    """
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
    """

    def rotate_around_ball(self):
        print "rotating around ball"

        angV = Vector3(15, 0, 0)
        linV = Vector3(2, 90, 0) ## TODO find proper vectors
        twistmsg = Twist(linV, angV)
        self.pub.publish(twistmsg)

    def rotate_around_ball_left(self):
        print "rotating around ball"
        angV = Vector3(2, 0, 0)
        linV = Vector3(-2, 270, 0) ## TODO find proper vectors
        twistmsg = Twist(linV, angV)
        self.pub.publish(twistmsg)

    def rotate_around_ball_right(self):
        print "rotating around ball"
        angV = Vector3(2, 0, 0)
        linV = Vector3(-2, 90, 0) ## TODO find proper vectors
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
        print "logic"
        game_logic.main_logic()
        rate.sleep()



