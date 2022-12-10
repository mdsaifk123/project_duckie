#!/usr/bin/env python3

import sys, math
import numpy as np
import rospy

from pid_control import PIDController

from geometry_msgs.msg import PoseStamped

from duckietown_msgs.msg import AprilTagDetectionArray

from duckietown_msgs.msg import Twist2DStamped
from duckietown_msgs.msg import FSMState

class AprilControl:
    # input_fsm_mode = "fsm_node/mode"
    input_tag_detections = "/instructobot00/apriltag_detector_node/detections"
    # output_car_cmd = "lane_controller_node/car_cmd"
    output_car_cmd = "/instructobot00/car_cmd_switch_node/cmd"

    # desired angle and position measurements
    desired_yaw = 0.0
    desired_phi = 0.0
    desired_z_trans = 0.1

    min_angle_error = 0.3
    min_linear_error = 0.01

    def __init__(self):
        # manually tuned
        # initialize the PID class
        self.control_signal = PIDController()
        #self.linear_controller = PIDController()

        # bind the input/output publishers and subcripbers
        self.bind_io()

    def bind_io(self):
        # subscribe the callback functions to the designated input topic
        rospy.Subscriber(self.input_tag_detections, AprilTagDetectionArray, self.receive_tag_detections)
        # subcribe various publishers to each output topic
        self.pub = rospy.Publisher(self.output_car_cmd, Twist2DStamped, queue_size=10)

    def receive_tag_detections(self, msg):
        detections = msg.detections
        # rospy.loginfo("april tag controller node received detections: " + str(detections))

        if(len(detections) > 0):
            observed_x_trans = detections[0].transform.translation.x
            observed_z_trans = detections[0].transform.translation.z
            observed_yaw = detections[0].transform.rotation.y

            observed_phi = math.atan(observed_x_trans/observed_z_trans)

            rospy.loginfo("april tag controller node received depth of tag: " + str(observed_z_trans))
            rospy.loginfo("april tag controller node received horizontal trans of tag: " + str(observed_x_trans))
            
            depth_error = self.desired_z_trans - observed_z_trans
            phi_error = self.desired_phi - observed_phi
            yaw_error = self.desired_yaw - observed_yaw


            # rospy.loginfo("april tag controller node received yaw error: " + str(yaw_error))
            rospy.loginfo("april tag controller node received phi error: " + str(phi_error))
            rospy.loginfo("april tag controller node received depth error: " + str(depth_error))
            rospy.loginfo("april tag controller node received yaw error: " + str(yaw_error))

            # angle_error = yaw_error + phi_error
            angle_error = phi_error
            linear_error = depth_error
            self.handle_errors(angle_error, linear_error)

        else:
            self.publish_car_cmd(0, 0)


    def publish_car_cmd(self, v, omega):
        # create the output data object
        output_data = Twist2DStamped()
        output_data.v = v
        output_data.omega = omega
        # publish the car command to the output topic
        self.pub.publish(output_data)


    def handle_errors(self, angle_error, linear_error):
        omega_signal = 0.0
        v_signal = 0.0
        if(abs(angle_error) > self.min_angle_error):
            Kp = 1.8
            Ki = 0.01
            Kd = 0.02
            omega_signal = self.control_signal.get_control_signal_from_error(angle_error, Kp, Ki, Kd)
        if(abs(linear_error) > self.min_linear_error):
            Kp = 0.75
            Ki = 0.003
            Kd = 0.06
            v_signal = self.control_signal.get_control_signal_from_error(linear_error, Kp, Ki, Kd)
        self.publish_car_cmd(-v_signal, omega_signal)

        
if __name__ == '__main__':
    rospy.init_node('april_control')
    a = AprilControl()
    # keep python from exiting
    rospy.spin()
