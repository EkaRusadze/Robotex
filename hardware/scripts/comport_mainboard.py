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
        self.refe_comm_helper = RefereeCommandsHelper()

    def monitoring_pub_callback(self, arg):
        msg = KeyValue()
        msg.MsgType = "mainboard"
        msg.MsgKey = "mainboard_connecter"
        msg.MsgValue = str(self.last_msg_received)
        self.gui_publisher.publish(msg)

    def message_received(self, recv_msg):
        self.last_msg_received = recv_msg

        msg_type, value = recv_msg.split(COMMAND_SEPARATOR, 1)
        msg_type = msg_type[1:]
        value = value[:-1]

        if msg_type == COMMAND_REF:
            rospy.loginfo("Command module raw message: {}".format(value))
            command = self.refe_comm_helper.receive_command(value)
            if len(command) > 0:
                self.send_ack()
                self.publish_command(command, msg_type)
        else:
            self.publish_command(value, msg_type)

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
                    self.connection_opened = self.connection.isOpen()
                    time.sleep(0.5)
                self.connection.flush()
                print "mainboard: Port opened successfully"
            except Exception as e:
                print(e)
                continue

        return self.connection_opened

    def write(self, comm):
        if self.connection is not None:
            try:
                self.connection.write(comm)
                # So appearantly we need to clear the read buffer to
                # the main board after writing, so we do that here
                #while self.connection.read() != '\n':
                #    pass
                self.connection.flush()
            except:
                print('mainboard: err write ' + comm)

    def get_speeds(self):
        self.write("gs\n")
        return self.connection.readline()

    def write_speeds(self):
        if self.connection is not None and self.connection_opened:
            rospy.loginfo("Speeds are: " + self.get_speeds())

    def toggle_red_led(self):
        if self.connection is not None and self.connection_opened:
            self.write("r\n")

    def set_motors(self, motor_one, motor_two, motor_three, motor_four):
        if self.connection_opened:
            self.write("sd:{}:{}:{}:{}\n".format(motor_one, motor_two,
                                                   motor_three, motor_four))

    def set_wheels(self, wheel_one, wheel_two, wheel_three):
        rospy.loginfo("Setting wheels to {} {} {}".format(wheel_one, wheel_two, wheel_three))
        self.set_motors(wheel_one, wheel_two, wheel_three, 0)

    def set_throw(self, speed):
        self.write("d:{}".format(speed))

    def close(self):
        if self.connection is not None and self.connection.isOpen():  # close coil
            try:
                self.connection.close()
                print('mainboard: connection closed')
            except:
                print('mainboard: err connection close')
            self.connection = None

    def run(self):
        if self.open():  # open serial connections
            print('mainboard: opened')
        else:
            print('mainboard: opening failed')
            self.close()
            return
