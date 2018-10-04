#!/usr/bin/env python
import rospy
from std_msgs.msg import String
import serial
from rattakiirus import wheelCalc, mainboardSpeedCalc
from geometry_msgs.msg import Twist



ser = serial.Serial(
    port='/dev/ttyACM0',
    baudrate=115200,
    dsrdtr=True,
    timeout=0.8
)


def movement(speed, angle, angularVelocity):
    #print speed, angle, angularVelocity
    wheelSpeed0 = str(mainboardSpeedCalc(wheelCalc(0, speed, angle, angularVelocity)))
    wheelSpeed1 = str(mainboardSpeedCalc(wheelCalc(1, -speed, angle, angularVelocity)))
    wheelSpeed2 = str(mainboardSpeedCalc(wheelCalc(2, speed, angle, angularVelocity)))
    print wheelSpeed0, wheelSpeed1, wheelSpeed2

    ser.write("sd:{0}:{1}:{2}:0\n".format(wheelSpeed0, wheelSpeed1, wheelSpeed2))

    ser.flush()



def twist_callback(twist):
    if twist is None:
        print 'ffs'
        return
    #print twist
    speed = twist.linear.x
    angle = twist.linear.y
    angVel = twist.angular.x
    movement(speed, angle, angVel)


# def blue_twist_callback(twist):
#     if twist is None:
#         print 'ffs'
#         return
#     print twist
#     speed = twist.linear.x
#     angle = twist.linear.y
#     angVel = twist.angular.x
#     movement(speed, angle, angVel)

# def magenta_twist_callback(twist):
#     if twist is None:
#         print 'ffs'
#         return
#     print twist
#     speed = twist.linear.x
#     angle = twist.linear.y
#     angVel = twist.angular.x
#     movement(speed, angle, angVel)

# es ro saertod ar imushavebs magashi echvi vinmes xo ar epareba
# teta rit mevikla tavi

if __name__ == '__main__':
    try:
        rospy.init_node("serial_comm")
        rospy.Subscriber("moving_vectors", Twist, twist_callback)
        #rospy.Subscriber("blue_vectors", Twist, blue_twist_callback)
        #rospy.Subscriber("magenta_vectors", Twist, magenta_twist_callback)
        #nah mate
        rate = rospy.Rate(30)
        while not rospy.is_shutdown():

            #rospy.Subscriber("moving_vectors", Twist, twist_callback)
            #twist_callback(twist_callback)
            #movement(speed, angle, angVel)
            rate.sleep()
    except rospy.ROSInterruptException:

        pass
