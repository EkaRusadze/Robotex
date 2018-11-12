#!/usr/bin/env python

import rospy
from std_msgs.msg import String
from geometry_msgs.msg import Point
import numpy as np
import cv2
import pyrealsense2 as rs
import time

#hsv_lower = (5, 110, 120)
#hsv_upper = (30, 150, 140)

hsv_lower = (19, 36, 45)
hsv_upper = (98, 173, 195)

blue_hsv_lower = (90, 200, 100)
blue_hsv_upper = (125, 255, 255)

magenta_hsv_lower = (158, 152, 100)
magenta_hsv_upper = (176, 192, 167)

class ImageProcessing():
    def __init__(self):
        self.pipeline = None
        self.contourRect = None
        self.align = None
        self.depth_image = None
        self.regular_image = None
        self.hsv_image = None
        self.bluecontourRect = None
        self.magentacontourRect = None

    def run(self):
        self.pipeline = rs.pipeline()
        config = rs.config()
        config.enable_stream(rs.stream.depth, 640, 480, rs.format.z16, 60)
        config.enable_stream(rs.stream.color, 640, 480, rs.format.bgr8, 60)
        self.pipeline.start(config)

        ctx = rs.context()
        devices = ctx.query_devices()
        for dev in devices:
            if dev.supports(rs.camera_info.product_id) and dev.supports(rs.camera_info.name):
                rospy.loginfo("Found camera device: {}".format(dev.get_info(rs.camera_info.name)))
                sensors = dev.query_sensors()
                for sensor in sensors:
                    if sensor.get_info(rs.camera_info.name) == "RGB Camera" and sensor.supports(
                            rs.option.exposure) and sensor.supports(rs.option.gain):
                        rospy.loginfo("Setting RGB camera sensor settings")
                        sensor.set_option(rs.option.enable_auto_exposure, 1)
                        sensor.set_option(rs.option.white_balance, 3000)
                        sensor.set_option(rs.option.enable_auto_white_balance, 0)
                        rospy.loginfo("exposure: {}".format(sensor.get_option(rs.option.exposure)))
                        rospy.loginfo("white balance: {}".format(sensor.get_option(rs.option.white_balance)))
                        rospy.loginfo("gain: {}".format(sensor.get_option(rs.option.gain)))
                        rospy.loginfo(
                            "auto exposure enabled: {}".format(sensor.get_option(rs.option.enable_auto_exposure)))
                        time.sleep(2)
                        sensor.set_option(rs.option.enable_auto_exposure, 0)
                        rospy.loginfo(
                            "auto exposure enabled: {}".format(sensor.get_option(rs.option.enable_auto_exposure)))
                        rospy.loginfo("exposure: {}".format(sensor.get_option(rs.option.exposure)))  # 166
                        rospy.loginfo("white balance: {}".format(sensor.get_option(rs.option.white_balance)))
                        rospy.loginfo("gain: {}".format(sensor.get_option(rs.option.gain)))  # 64
                        break
                break

        rospy.loginfo("REALSENSE camera successfully initialized!")

        align_to = rs.stream.color
        self.align = rs.align(align_to)

    def get_frame(self):
        frames = self.pipeline.wait_for_frames()
        aligned_frames = self.align.process(frames)

        aligned_depth_frame = aligned_frames.get_depth_frame()  # aligned_depth_frame is a 640x480 depth image
        color_frame = aligned_frames.get_color_frame()

        if not aligned_depth_frame or not color_frame:
            return

        self.depth_image = np.asanyarray(aligned_depth_frame.get_data())
        self.regular_image = np.asanyarray(color_frame.get_data())
        self.hsv_image = cv2.cvtColor(self.regular_image, cv2.COLOR_BGR2HSV)
        #print self.hsv_image[320][240]
        # cv2.imshow("image", self.hsv_image)
        # cv2.waitKey(1)
        #
        # # Creating a window for later use
        # cv2.namedWindow('result')
        #
        # def nothing(x):
        #     pass
        #
        # # Starting with 100's to prevent error while masking
        # h, s, v = 100, 100, 100
        #
        # # Creating track bar
        # cv2.createTrackbar('h', 'result', 0, 179, nothing)
        # cv2.createTrackbar('s', 'result', 0, 255, nothing)
        # cv2.createTrackbar('v', 'result', 0, 255, nothing)
        # cv2.createTrackbar('hh', 'result', 0, 179, nothing)
        # cv2.createTrackbar('hs', 'result', 0, 255, nothing)
        # cv2.createTrackbar('hv', 'result', 0, 255, nothing)
        #
        # # get info from track bar and appy to result
        # h = cv2.getTrackbarPos('h', 'result')
        # s = cv2.getTrackbarPos('s', 'result')
        # v = cv2.getTrackbarPos('v', 'result')
        # hh = cv2.getTrackbarPos('hh', 'result')
        # hs = cv2.getTrackbarPos('hs', 'result')
        # hv = cv2.getTrackbarPos('hv', 'result')
        #
        # # Normal masking algorithm
        # lower_blue = np.array([h, s, v])
        # upper_blue = np.array([hh, hs, hv])
        #
        # mask = cv2.inRange(self.hsv_image, lower_blue, upper_blue)
        #
        # result = cv2.bitwise_and(self.hsv_image, self.hsv_image, mask=mask)
        #
        # cv2.imshow('result', result)
        # cv2.waitKey(1)




    def detect_contours(self):
        mask = cv2.inRange(self.hsv_image, hsv_lower, hsv_upper)
        result = cv2.bitwise_and(self.hsv_image, self.hsv_image, mask=mask)
        #cv2.imshow("maskball", result)
        #cv2.waitKey(1)
        #cv2.imshow("hsv_image", self.hsv_image)
        # Bitwise-AND mask and original image
        im2, contours, hierarchy = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
        if len(contours) != 0:
            #print len(contours)
            self.contourRect = []
            # find the biggest area
            for contour in contours:
                contArea = cv2.contourArea(contour)
                if contArea > 20:
                    #print "slksdlfkdljg"
                    rect = cv2.boundingRect(contour)
                    #rect = x, y, w, h
                    self.contourRect.append(rect)

            # closestBall = max(contourArea)
            # closestBall = np.array(closestBall, dtype=np.float32)
            # self.rect = cv2.boundingRect(closestBall)
    def detect_blue_basket(self):
        mask = cv2.inRange(self.hsv_image, blue_hsv_lower, blue_hsv_upper)
        result = cv2.bitwise_and(self.hsv_image, self.hsv_image, mask=mask)
        # cv2.imshow("mask", result)
        # cv2.waitKey(1)
        # cv2.imshow("hsv_image", self.hsv_image)
        # Bitwise-AND mask and original image
        im2, contours, hierarchy = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
        if len(contours) != 0:
            # print len(contours)
            self.bluecontourRect = []
            # find the biggest area
            for contour in contours:
                contArea = cv2.contourArea(contour)
                if contArea > 20:
                    # print "slksdlfkdljg"
                    rect = cv2.boundingRect(contour)
                    # rect = x, y, w, h
                    self.bluecontourRect.append(rect)

            # closestBall = max(contourArea)
            # closestBall = np.array(closestBall, dtype=np.float32)
            # self.rect = cv2.boundingRect(closestBall)

    def detect_magenta_basket(self):
            mask = cv2.inRange(self.hsv_image, magenta_hsv_lower, magenta_hsv_upper)
            result = cv2.bitwise_and(self.hsv_image, self.hsv_image, mask=mask)
            #cv2.imshow("maskbasket", result)
            #cv2.waitKey(1)
            # cv2.imshow("hsv_image", self.hsv_image)
            # Bitwise-AND mask and original image
            im2, contours, hierarchy = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
            if len(contours) != 0:
                # print len(contours)
                self.magentacontourRect = []
                # find the biggest area
                for contour in contours:
                    contArea = cv2.contourArea(contour)
                    if contArea > 20:
                        # print "slksdlfkdljg"
                        rect = cv2.boundingRect(contour)
                        # rect = x, y, w, h
                        self.magentacontourRect.append(rect)

    def ball_coordinates(self):
        if self.contourRect != None:
            if len(self.contourRect) > 0:
                max_size = 0
                max_ball = None
                for (x, y, width, height) in self.contourRect:
                    if (width * height) > max_size:
                        max_size = width * height
                        max_ball = (x, y, width, height)

                if max_ball is not None:
                    #print max_ball
                    (x, y, width, height) = max_ball
                    #print x, y, width, height
                    xcoord = int(x+width/2)
                    ycoord = int(y+height/2)
                    print "ball coordinates:", xcoord, ycoord

                    coordinates = rospy.Publisher("ball_coordinates", Point, queue_size=10)
                    coordinates.publish(Point(xcoord, ycoord, 0))

    def blue_basket_coordinates(self):
        if self.bluecontourRect != None:
            if len(self.bluecontourRect) > 0:
                max_size = 0
                max_basket = None
                for (x, y, width, height) in self.bluecontourRect:
                    if (width * height) > max_size:
                        max_size = width * height
                        max_basket = (x, y, width, height)

                if max_basket is not None:
                    #print max_basket
                    (x, y, width, height) = max_basket
                    #print x, y, width, height
                    xcoord = int((x+width)/2)
                    ycoord = int(y + height / 2)
                    print "coordinates:", xcoord, ylow, yhigh

                    coordinates = rospy.Publisher("basket_coordinates", Point, queue_size=10)
                    coordinates.publish(Point(xcoord, ycoord))

    def magenta_basket_coordinates(self):
        if self.magentacontourRect != None:
            if len(self.magentacontourRect) > 0:
                max_size = 0
                max_basket = None
                for (x, y, width, height) in self.magentacontourRect:
                    if (width * height) > max_size:
                        max_size = width * height
                        max_basket = (x, y, width, height)

                if max_basket is not None:
                    #print max_basket
                    (x, y, width, height) = max_basket
                    #print x, y, width, height
                    xcoord = int((x+width)/2)
                    ycoord = int(y + height / 2)
                    #ylow = y + height
                    #yhigh = y
                    #print "magenta coordinates:", xcoord, ylow, yhigh

                    coordinates = rospy.Publisher("basket_coordinates", Point, queue_size=10)
                    coordinates.publish(Point(xcoord, ycoord))

if __name__ == "__main__":
    try:
        rospy.init_node("receive_image")
        camera = ImageProcessing()
        camera.run()
        rate = rospy.Rate(60)

        while not rospy.is_shutdown():
            camera.get_frame()
            camera.detect_contours()
            #camera.detect_blue_basket()
            camera.detect_magenta_basket()
            camera.ball_coordinates()
            #camera.blue_basket_coordinates()
            camera.magenta_basket_coordinates()
            rate.sleep()

    except rospy.ROSInterruptException:
        pass