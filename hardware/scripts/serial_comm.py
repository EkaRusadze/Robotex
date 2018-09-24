#!/usr/bin/env python
import rospy
from std_msgs.msg import String
import serial
from rattakiirus import wheelCalc, mainboardSpeedCalc



ser = serial.Serial(
    port='/dev/ttyACM0',
    baudrate=115200,
    parity=serial.PARITY_NONE,
    stopbits=serial.STOPBITS_TWO,
    bytesize=serial.EIGHTBITS
)


def movement(speed, angle, angularVelocity):
    wheelSpeed0 = str(mainboardSpeedCalc(wheelCalc(0, speed, angle, angularVelocity)))
    wheelSpeed1 = str(mainboardSpeedCalc(wheelCalc(1, speed, angle, angularVelocity)))
    wheelSpeed2 = str(mainboardSpeedCalc(wheelCalc(2, speed, angle, angularVelocity)))

    ser.write("sd:{0}:{1}:{2}:0\n".format(wheelSpeed0, wheelSpeed1, wheelSpeed2).encode())
    #ser.write("sd:{0}:{1}:{2}:0\n".format(30, 30, 30).encode())

    print '123213'
    #b = ser.read()





if __name__ == '__main__':
    try:
        rospy.init_node("serial_comm")
        rate = rospy.Rate(30)
        while not rospy.is_shutdown():
            print 'aslkds'
            movement(0.1, 0, 0)
            rate.sleep()
    except rospy.ROSInterruptException:

        pass
