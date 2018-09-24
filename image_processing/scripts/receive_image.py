#!/usr/bin/env python

import rospy
from std_msgs.msg import String
from geometry_msgs.msg import Point
import numpy as np
import cv2
import pyrealsense2 as rs

hsv_lower = (60, 100, 40)
hsv_upper = (90, 255, 255)

class ImageProcessing():
    def __init__(self):
        self.pipeline = None
        self.contourRect = None
        self.align = None
        self.depth_image = None
        self.regular_image = None
        self.hsv_image = None

    def run(self):
        self.pipeline = rs.pipeline()
        config = rs.config()
        config.enable_stream(rs.stream.depth, 640, 480, rs.format.z16, 60)
        config.enable_stream(rs.stream.color, 640, 480, rs.format.bgr8, 60)
        self.pipeline.start(config)
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
<<<<<<< HEAD
        cv2.imshow("image", self.hsv_image)



    def detect_contours(self):
        mask = cv2.inRange(self.hsv_image, hsv_lower, hsv_upper)
        cv2.imshow("mask", mask)
        #cv2.imshow("hsv_image", self.hsv_image)
        # Bitwise-AND mask and original image
        im2, contours, hierarchy = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
        if len(contours) != 0:
            #print len(contours)
            self.contourRect = []
=======



    def detect_contours(self): #arguments?????
        mask = cv2.inRange(self.hsv_image, hsv_lower, hsv_upper)
        # Bitwise-AND mask and original image
        res = cv2.bitwise_and(self.hsv_image, self.hsv_image, mask=mask)
        im2, contours, hierarchy = cv2.findContours(mask, cv2.RETR_TREE, cv2.CHAIN_APPROX_SIMPLE)
        if len(contours) != 0:
            #print len(contours)
            self.contourRect = list()
>>>>>>> 1733b9a8dd1cadc285b4b2f3e8f6677163274332
            # find the biggest area
            for contour in contours:
                contArea = cv2.contourArea(contour)
                if contArea > 20:
                    #print "slksdlfkdljg"
                    contArea = np.array(contArea, dtype=np.uint8)
                    rect = cv2.boundingRect(contArea)
                    #rect = x, y, w, h
                    self.contourRect.append(rect)

            # closestBall = max(contourArea)
            # closestBall = np.array(closestBall, dtype=np.float32)
            # self.rect = cv2.boundingRect(closestBall)

    def ball_coordinates(self):
<<<<<<< HEAD
=======
        try:
            self.contourRect
        except:
            self.contourRect = None
>>>>>>> 1733b9a8dd1cadc285b4b2f3e8f6677163274332
        if self.contourRect != None:
            if len(self.contourRect) > 0:
                max_size = 0
                max_ball = ()
                for (x, y, width, height) in self.contourRect:
                    if (width * height) > max_size:
                        max_size = width * height
<<<<<<< HEAD
                        max_ball = (x, y, width, height)

=======

                        max_ball = (x, y, width, height)
>>>>>>> 1733b9a8dd1cadc285b4b2f3e8f6677163274332
                print max_ball
                (x, y, width, height) = max_ball
                print x, y, width, height
                xcoord = int(width/2)
                ycoord = int(height/2)
                print "coordinates:", xcoord, ycoord

                coordinates = rospy.Publisher("ball_coordinates", Point, queue_size=10)
                coordinates.publish(Point(x, y, 0))

if __name__ == "__main__":
    try:
        rospy.init_node("receive_image")
<<<<<<< HEAD
        camera = ImageProcessing()
        camera.run()
        rate = rospy.Rate(60)

        while not rospy.is_shutdown():


=======

        rate = rospy.Rate(60)

        while not rospy.is_shutdown():
            camera = ImageProcessing()
            camera.run()
>>>>>>> 1733b9a8dd1cadc285b4b2f3e8f6677163274332
            camera.get_frame()
            camera.detect_contours()
            camera.ball_coordinates()
            rate.sleep()

    except rospy.ROSInterruptException:
        pass