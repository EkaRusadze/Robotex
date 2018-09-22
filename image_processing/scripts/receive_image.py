import rospy
from std_msgs.msg import String
import numpy as np
import cv2
import pyrealsense2 as rs

hsv_lower = np.array(48, 165, 90)
hsv_upper = np.array(75, 255, 255)

class ImageProcessing():
    def run(self):
        self.pipeline = rs.pipeline()
        config = rs.config()
        config.enable_stream(rs.stream.depth, conf.DEPTH_WIDTH, conf.DEPTH_HEIGHT, rs.format.z16, 60)
        config.enable_stream(rs.stream.color, conf.WIDTH, conf.HEIGHT, rs.format.bgr8, 60)
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
        self.hsv_image = cv2.cvtColor(self.regular_image, cv2.COLOR_BGR2YUV)



    def detect_contours(self, hsv_image): #arguments?????
        mask = cv2.inRange(hsv_image, hsv_lower, hsv_upper)
        # Bitwise-AND mask and original image
        res = cv2.bitwise_and(frame, frame, mask=mask)
        im2, contours, hierarchy = cv2.findContours(mask, cv2.RETR_TREE, cv2.CHAIN_APPROX_SIMPLE)
        if len(contours) != 0:
            contourArea = []
            # find the biggest area
            for i in contours:
                contArea = cv2.contourArea(contour)
                contourArea.append(contArea)

            closestBall = max(contourArea)
            rect = cv2.boundingRect(closestBall)

    def ball_coordinates(self, rect):
        height, width = rect.shape[:2]
        x = int(height/2)
        y = int(width/2)

        coordinates = rospy.Publisher("ball_coordinates", Point, queue_size=10)
        coordinates.publish(Point(x, y, 0))
