#!/usr/bin/env python3

import sys, math
import numpy as np
import rospy
import cv2


from pid_controller import PIDController

# from geometry_msgs.msg import PoseStamped

from duckietown_msgs.msg import Twist2DStamped
from duckietown_msgs.msg import LanePose
from duckietown_msgs.msg import FSMState

class lanecontroller:
    desired_d = 0.0
    desired_phi = 0.0

    min_phi_error = 0.1
    min_d_error = 0.05

    following = False

    input_fsm_mode = "fsm_node/mode"
    input_lane_pose = "lane_filter_node/lane_pose"

    # output_car_cmd = "lane_controller_node/car_cmd"
    output_car_cmd = "car_cmd_switch_node/cmd"

    def __init__(self):
        self.angle_pid = PIDController()
        # bind the input/output publishers and subcripbers
        self.bind_io();

    def bind_io(self):
        # subscribe the callback functions to the designated input topic
        rospy.Subscriber(self.input_fsm_mode, FSMState, self.receive_fsm_mode)
        rospy.Subscriber(self.input_lane_pose, LanePose, self.receive_lane_pose)
        # subcribe various publishers to each output topic
        self.pub = rospy.Publisher(self.output_car_cmd, Twist2DStamped, queue_size=10)

    def receive_lane_pose(self, msg):
        # if(self.following):
        rospy.loginfo("lane controller node received msg: " + str(msg))
        observed_d = msg.d
        observed_phi = msg.phi
        
        distance_error = self.desired_d - observed_d
        phi_error = self.desired_phi - observed_phi

        self.handle_errors(phi_error, distance_error)

    def publish_car_cmd(self, v, omega):
        # create the output data object
        output_data = Twist2DStamped()
        output_data.v = v
        output_data.omega = omega
        # publish the car command to the output topic
        self.pub.publish(output_data)


    def handle_errors(self, phi_error, distance_error):
        angle_error = 0
        if(abs(distance_error) > self.min_d_error):
            angle_error = angle_error + distance_error
        if(abs(phi_error) > self.min_phi_error):
            angle_error = angle_error + phi_error

        # manually tuned
        Kp = 3.6
        Ki = 0.0
        Kd = 0.02
        omega_signal = self.angle_pid.get_control_signal_from_error(angle_error, Kp, Ki, Kd)
        self.publish_car_cmd(0.24, omega_signal)

    def receive_fsm_mode(self, msg):
        rospy.loginfo("lane-controller-node received msg from fsm_node: " + str(msg))
        if msg.state == "LANE_FOLLOWING":
            # perform lane following procedure
            self.following = True
        else:
            self.following = False
        
if __name__ == '__main__':
    rospy.init_node('lane_controller')
    lanecontroller()
    # keep python from exiting
    rospy.spin()
