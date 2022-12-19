#!/usr/bin/env python3

from math import sin, cos
import time
import numpy as np
import rospy

from duckietown_msgs import WheelEncodeStamped
from duckietown_msgs import Pose2DStamped

from odometry_hw.msg import DistWheel
from odometry_hw.msg import Pose2D

class Odometry_lab2:

    input_topic_right = "/instructobot00/right_wheel_encoder_node/tick"
    input_topic_left = "/instructobot00/left_wheel_encoder_node/tick"
    output_topic = "/instructobot00/velocity_to_pose_node/pose"

    dist_between_wheels = 0.1 # in meters
    last_measure_time = 0.0
    current_pose = Pose2DStamped()

    def __init__(self):
        # subscribe the callback function to the designated input topic
        rospy.Subscriber(self.input_topic_right, WheelEncodeStamped, self.callback)
        rospy.Subscriber(self.input_topic_left, WheelEncodeStamped, self.callback)
        # set the last measured timestamp to the initialization time
        self.last_measure_time = time.time()

        self.pose_pub = rospy.Publisher(self.output_topic, Pose2DStamped, queue_size=10)

    def update_pose(self, WheelEncodeStamped):
        # get the current time
        ts = time.time()
        # calculate the delta_t since last measurement
        delta_t = ts - self.last_measure_time

        # calculate the change in arc length and angle 
        delta_s = self.calculate_change_in_arc_from_wheel_dist(WheelEncodeStamped)
        delta_theta = self.calculate_change_in_angle_from_wheel_dist(WheelEncodeStamped)

        # calculate the change in robot position from these
        delta_x, delta_y = self.calculate_robot_position_change(delta_s, delta_theta)

        # update the current robot pose
        self.update_pose(delta_x, delta_y, delta_theta, delta_t)

        # update the timestamp
        self.last_measure_time = ts


    def update_pose(self, delta_x, delta_y, delta_theta, delta_t):
        self.current_pose.x = self.current_pose.x + delta_x
        self.current_pose.y = self.current_pose.y + delta_y
        self.current_pose.theta = self.current_pose.theta + delta_theta
        rospy.loginfo("odometry node updated pose after time: " + str(delta_t) + " to: \n" + str(self.current_pose))


    def calculate_change_in_arc_from_wheel_dist(self, WheelEncodeStamped):
        # extract the left and right wheel distance data
        l_dist = WheelEncodeStamped.data
        r_dist = WheelEncodeStamped.data
        delta_s = (l_dist + r_dist) / 2
        return  delta_s

    def calculate_change_in_angle_from_wheel_dist(self, WheelEncodeStamped):
        # extract the left and right wheel distance data
        l_dist = WheelEncodeStamped.data 
        r_dist = WheelEncodeStamped.data
        delta_theta = (r_dist - l_dist) / self.dist_between_wheels
        return  delta_theta

    def calculate_robot_position_change(self, delta_s, delta_theta):
        theta = self.current_pose.theta
        delta_x = delta_s*cos(theta + delta_theta/2)
        delta_y = delta_s*sin(theta + delta_theta/2)
        return delta_x, delta_y

    def publish_current_pose(self):
        # create the output data object
        output_data = self.current_pose 
        # publish the robot pose data to the output topic
        self.pose_pub.publish(output_data)

    def callback(self, msg):
        rospy.loginfo("odometry node heard wheel distances from sensor: " + str(msg.dist_wheel_left) + " " + str(msg.dist_wheel_right))
        # extract the wheel distance 
        WheelEncodeStamped = msg
        # update the robot pose (positoion + orientation) with the new wheel distance info
        self.update_pose_from_dist_wheel(WheelEncodeStamped)
        # publish the updated pose
        self.publish_current_pose()

if __name__ == '__main__':
    rospy.init_node('odometry_lab2')
    Odometry_lab2()
    # keep python from exiting
    rospy.spin()
