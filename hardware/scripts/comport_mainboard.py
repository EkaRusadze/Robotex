#!/usr/bin/env python
# Taken or adapted from
# https://bitbucket.org/MartinAppo/diploaf-2017/src/master/hardware_module/src/hardware_module/comport_mainboard.py

import serial
import threading
import time
import subprocess
import rospy


class ComportMainboard(threading.Thread):
    connection = None
    connection_opened = False

    def __init__(self):
        threading.Thread.__init__(self)

    def open(self):
        try:
            ports = subprocess.check_output('ls /dev/ttyACM0', shell=True).split('\n')[:-1]
        except:
            print('mainboard: /dev/ttyACM empty')
            return False
        self.connection_opened = False
        for port in ports:  # analyze serial ports
            try:
                while not self.connection_opened and not rospy.is_shutdown():
                    self.connection = serial.Serial(port, baudrate=115200, timeout=0.8, dsrdtr=True)
                    self.connection_opened = self.connection.is_open
                    time.sleep(0.5)
                    self.connection.flush()
                    print "mainboard: Port opened successfully"
            except Exception as e:
                print(e)
                continue

        return self.connection_opened

    def write(self, comm):
        print "comm write"
        if self.connection is not None:
            try:
                print "try write"
                self.connection.write(comm)
                # So appearantly we need to clear the read buffer to
                # the main board after writing, so we do that here
                #while self.connection.read() != '\n':
                #    pass
                #self.connection.flush()
            except:
                print('mainboard: err write ' + comm)
        else:
            print "comm write is None"

    def get_speeds(self):
        self.write("gs\n")
        return self.connection.readline()

    def write_speeds(self):
        if self.connection is not None and self.connection_opened:
            rospy.loginfo("Speeds are: " + self.get_speeds())

    def toggle_red_led(self):
        if self.connection is not None and self.connection_opened:
            self.write("r\n")

    def set_motors(self, motor_one, motor_two, motor_three):
        if self.connection is not None and self.connection.is_open:
            print "set_motors"
            self.write("sd:{}:{}:{}\n".format(motor_one, motor_two, motor_three))
        else:
            print "connection closed set motors"

    def set_wheels(self, wheel_one, wheel_two, wheel_three):
        rospy.loginfo("Setting wheels to {} {} {}".format(wheel_one, wheel_two, wheel_three))
        self.set_motors(wheel_one, wheel_two, wheel_three)

    def set_throw(self, speed):
        self.write("d:{}\n".format(speed))

    def close(self):
        if self.connection is not None and self.connection.is_open:  # close coil
            try:
                self.connection.close()
                print('mainboard: connection closed')
            except:
                print('mainboard: err connection close')
            self.connection = None

    def read_mainboard(self):
        if self.connection is not None and self.connection.is_open:
            while self.connection.in_waiting > 0:
                self.connection.read()


    def run(self):
        if self.open():  # open serial connections
            print('mainboard: opened')
        else:
            print('mainboard: opening failed')
            self.close()
            return
