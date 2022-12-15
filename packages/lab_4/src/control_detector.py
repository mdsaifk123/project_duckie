#!/usr/bin/env python3

import sys, math
import numpy as np
import rospy
import cv2

from sensor_msgs.msg import Image
from sensor_msgs.msg import CompressedImage

from duckietown_msgs.msg import SegmentList
from duckietown_msgs.msg import Segment

from cv_bridge import CvBridge

class CustomLaneDetectorNode:
    input_image_topic = "camera_node/image/compressed"

    output_edges_topic = "/image_edges"
    output_white_lines_topic = "/image_lines_white"
    output_yellow_lines_topic = "/image_lines_yellow"
    output_all_lines_topic = "/image_lines_all"

    output_segments_topic = "line_detector_node/segment_list"

    received_images = dict()

    def __init__(self):
        # initialize the OpenCV bridge
        self.bridge = CvBridge()
        # bind the input/output publishers and subcripbers
        self.bind_io()

    def bind_io(self):
        # subscribe the callback functions to the designated input topic
        rospy.Subscriber(self.input_image_topic, CompressedImage, self.receive_camera_image, queue_size=1, buff_size=2**24)

        # subcribe various publishers to each output topic
        self.pub_edges = rospy.Publisher(self.output_edges_topic, Image , queue_size=10)
        self.pub_yellow_lines = rospy.Publisher(self.output_yellow_lines_topic, Image , queue_size=10)
        self.pub_white_lines = rospy.Publisher(self.output_white_lines_topic, Image , queue_size=10)
        self.pub_all_lines = rospy.Publisher(self.output_all_lines_topic, Image , queue_size=10)

        self.pub_segments = rospy.Publisher(self.output_segments_topic, SegmentList , queue_size=10)


    def erode_image(self, image):
        # for errosion

        kernel = cv2.getStructuringElement(cv2.MORPH_ELLIPSE, (3,3))
        image_erode = cv2.erode(image, kernel)
        return image_erode

    def dialate_image(self, image):
        ## for dialation

        kernel = cv2.getStructuringElement(cv2.MORPH_ELLIPSE, (3,3))
        image_dialate = cv2.dilate(image, kernel)
        return image_dialate

    def crop_image(self, image):
        # get the width and height of the image

        image_size = (160, 120)
        offset = 40
        new_image = cv2.resize(image, image_size, interpolation=cv2.INTER_NEAREST)
        cropped_image = new_image[offset:, :]
        # return the cropped image data
        return cropped_image

    def hsv_convert_image(self, image):
        # convert the bgr image to hsv format
        hsv_image = cv2.cvtColor(image, cv2.COLOR_RGB2HSV)
        # return the result
        return hsv_image

    def filter_yellow_image(self, image):
        # define a range for the yellow color in HSV format
        # lower_yellow = np.array([22, 93, 0], dtype=np.uint8)
        lower_yellow = np.array([10, 20, 0], dtype=np.uint8)
        # upper_yellow = np.array([45, 255, 255], dtype=np.uint8)
        upper_yellow = np.array([45, 255, 255], dtype=np.uint8)
        # filter the yellow pixels out of the image 
        yellow_image = cv2.inRange(image, lower_yellow, upper_yellow)

        # apply erosion to the image to remove superflous residue
        for n in range(3):
            yellow_image = self.erode_image(yellow_image)
        # apply dialation to the image to remove fill gaps
        for n in range(4):
            yellow_image = self.dialate_image(yellow_image)

        # return the result
        return yellow_image

    def filter_white_image(self, image):
        # define a range for the white color in HSV format
        sensitivity = 80
        lower_white = np.array([0,0,255-sensitivity], dtype=np.uint8)
        upper_white = np.array([255,sensitivity,255], dtype=np.uint8)
        # filter the white pixels out of the image 
        white_image = cv2.inRange(image, lower_white, upper_white)

        # apply erosion to the image to remove superflous residue
        for n in range(1):
            white_image = self.erode_image(white_image)
        # apply dialation to the image to remove fill gaps
        for n in range(3):
            white_image = self.dialate_image(white_image)
        # return the result
        return white_image


    def publish_all_lines_image(self, lines_image):
        # convert the image data to ros format and publish it
        img_msg = self.bridge.cv2_to_imgmsg(lines_image, encoding="rgb8")
        self.pub_all_lines.publish(img_msg)

    def publish_white_lines_image(self, white_image):
        # convert the image data to ros format and publish it
        img_msg = self.bridge.cv2_to_imgmsg(white_image, encoding="rgb8")
        self.pub_white_lines.publish(img_msg)

    def publish_yellow_lines_image(self, yellow_image):
        # convert the image data to ros format and publish it
        img_msg = self.bridge.cv2_to_imgmsg(yellow_image, encoding="rgb8")
        self.pub_yellow_lines.publish(img_msg)

    def publish_image_edges(self, edges):
         # convert the image data to ros format and publish it
         img_msg = self.bridge.cv2_to_imgmsg(edges, encoding="mono8")
         self.pub_edges.publish(img_msg)

    def publish_segment_list(self, segment_list):
        self.pub_segments.publish(segment_list)

    def output_lines(self, original_image, lines, color):
        output = np.copy(original_image)
        if lines is not None:
            for i in range(len(lines)):
                l = lines[i][0]
                cv2.line(output, (l[0],l[1]), (l[2],l[3]), color, 2, cv2.LINE_AA)
                cv2.circle(output, (l[0],l[1]), 2, (0,255,0))
                cv2.circle(output, (l[2],l[3]), 2, (0,0,255))
        return output

    def process_images(self):
        cropped_image = self.received_images["cropped"]
        edges = self.received_images["edges"]
        white_image = self.received_images["white"]
        yellow_image = self.received_images["yellow"]

        # rospy.loginfo("yellow images are: " + str(yellow_image))
        
        # isolate white and yellow edges using color mask
        white_edges = cv2.bitwise_and(edges, edges, mask=white_image)
        yellow_edges = cv2.bitwise_and(edges, edges, mask=yellow_image)

        # apply the Hough transform to calculate lines from edge points
        threshold = 20
        white_lines = cv2.HoughLinesP(white_edges, 1, np.pi/180, threshold, minLineLength=5, maxLineGap=25)
        threshold = 1
        yellow_lines = cv2.HoughLinesP(yellow_edges, 1, np.pi/180, threshold, minLineLength=2, maxLineGap=5)

        # rospy.loginfo("white lines are: " + str(white_lines))
        # rospy.loginfo("yellow lines are: " + str(yellow_lines))

        # output these lines onto cropped image and publish the results
        yellow_lines_image = self.output_lines(cropped_image, yellow_lines, (0,0,255))
        white_lines_image = self.output_lines(cropped_image, white_lines, (255,0,0))
        all_lines_image = self.output_lines(yellow_lines_image, white_lines, (255,0,0))

        self.publish_yellow_lines_image(yellow_lines_image)
        self.publish_white_lines_image(white_lines_image)
        self.publish_all_lines_image(all_lines_image)

        # construct the normalized segment list
        white_segments = []
        # 0 for white color, 1 for yellow
        if(white_lines is not None): 
            white_segments = self.get_normalized_line_segments(white_lines, 0)
        yellow_segments = []
        if(yellow_lines is not None): 
            yellow_segments = self.get_normalized_line_segments(yellow_lines, 1)

        segment_list = SegmentList()
        segment_list.segments = white_segments + yellow_segments
        
        self.publish_segment_list(segment_list)



        # empty the received images data to receive new images
        self.received_images.clear()

    def get_normalized_line_segments(self, lines, color):
        img_size = (160, 120)
        offset = 40
        arr_cutoff = np.array([0, offset, 0, offset])
        arr_ratio = np.array([1. / img_size[0], 1. / img_size[1], 1. / img_size[0], 1. / img_size[1]])

        segment_list = []
        for line in lines:
            line_segment = Segment()
            line_normalized = (line + arr_cutoff) * arr_ratio
            line_normalized = line_normalized.flatten()
            # rospy.loginfo("line_normalized appears as: " + str(line_normalized))
            line_segment.pixels_normalized[0].x = line_normalized[0]
            line_segment.pixels_normalized[0].y = line_normalized[1]
            line_segment.pixels_normalized[1].x = line_normalized[2]
            line_segment.pixels_normalized[1].y = line_normalized[3]
            line_segment.color = color
            segment_list.append(line_segment)
        return segment_list

    def check_images_ready(self):
        expected_keys = ["cropped", "edges", "white", "yellow"]
        for key in expected_keys:
            if key not in self.received_images:
                return False
        return True

    def attempt_image_processing(self):
        images_ready = self.check_images_ready()
        if(images_ready):
            self.process_images()


    def handle_cropped_image(self, cv_image):
        # perform canny edge detection and publish the resultant edges
        gray = cv2.cvtColor(cv_image, cv2.COLOR_BGR2GRAY)
        edges = cv2.Canny(gray, 50, 200)
        # self.publish_image_edges(edges)
        # store the image in the recived_images array 
        self.received_images["cropped"] = cv_image
        self.received_images["edges"] = edges
        # attempt image processing
        self.attempt_image_processing()


    def handle_white_image(self, cv_image):
        # store the image in the recived_images array 
        self.received_images["white"] = cv_image
        # attempt image processing
        self.attempt_image_processing()


    def handle_yellow_image(self, cv_image):
        # store the image in the recived_images array 
        self.received_images["yellow"] = cv_image
        # attempt image processing
        self.attempt_image_processing()


    def receive_camera_image(self, msg):
        # extract the image from the msg data
        cv_image = self.bridge.compressed_imgmsg_to_cv2(msg, "rgb8")
        # # crop the image and publish the result
        cropped_image = self.crop_image(cv_image)
        self.handle_cropped_image(cropped_image)
        # convert the image to HSV format
        hsv_image = self.hsv_convert_image(cropped_image)
        # filter the hsv image for white and publish the resultt
        white_image = self.filter_white_image(hsv_image)
        self.handle_white_image(white_image)
        # # filter the hsv image for yellow and publish the resultt
        yellow_image = self.filter_yellow_image(hsv_image)
        self.handle_yellow_image(yellow_image)
        
if __name__ == '__main__':
    rospy.init_node('custom_lane_detector_node')
    CustomLaneDetectorNode()
    # keep python from exiting
    rospy.spin()
