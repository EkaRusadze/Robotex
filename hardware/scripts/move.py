#!/usr/bin/env python

import rospy
import serial
from std_msgs.msg import Int32MultiArray, Float32MultiArray, String, Int32
import time

ser = None
current_command = "START"
sdata = ""
stime = None
prevstime = None
gdata = ""


def init_mainboard():
    global ser
    print
    "initiating mainboard"
    ser = serial.Serial("/dev/ttyACM0", 9600, timeout=0.01, write_timeout=0.01)
    # ser.write('sd:0:0:0\n')


def turn_on_motors(data):
    global current_command, sdata, stime, ser
    if current_command == "STOP":
        while 1:
            try:
                ser.write('sd:0:0:0\n')
                break
            except serial.SerialTimeoutException:
                ser.close()
                init_mainboard()
        print
        "stopping motors"
    else:
        (f1, f2, f3) = tuple(data.data[0:3])
        while 1:
            try:
                ser.write('sd:%i:%i:%i\n' % (round(f1), round(f3), round(f2)))
                break
            except serial.SerialTimeoutException:
                ser.close()
                init_mainboard()
        print
        "turning motors at", f1, f2, f3




def __main__():
    #global current_command, gdata,

    rospy.Subscriber("ToMotors", Int32MultiArray, turn_on_motors)
    pub2 = rospy.Publisher("FromMotors", Float32MultiArray, queue_size=10)
    init_mainboard()
    ser.write('sd:0:0:0\n')
    rate = rospy.Rate(500)
    while not rospy.is_shutdown():
        turn_on_motors(Int32MultiArray(data=[8, 8, 8]))


        rate.sleep()


if __name__ == '__main__':
    try:
        rospy.init_node("move")
        while not rospy.is_shutdown():
            __main__()
            print "asdsfsgfd"
    except rospy.ROSInterruptException, e:
        print
        e
        if ser:
            ser.close()
    except serial.SerialException, e:
        print
        e
        if ser:
            ser.close()
    except KeyboardInterrupt, e:
        print
        e
        if ser:
            ser.close()
