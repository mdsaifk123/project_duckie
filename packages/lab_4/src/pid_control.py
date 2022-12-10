#!/usr/bin/env python3

import sys, math
import numpy as np
import rospy
import cv2

from sensor_msgs.msg import Image
from cv_bridge import CvBridge

from collections import deque

class PIDController:
    proportional_gain = 1.0 # K_p
    integral_gain = 1.0 # K_i
    derivative_gain = 1.0 # K_d

    error_total = 0.0

    def __init__(self, Kp, Ki, Kd):
        # initialize the error gains
        self.proportional_gain = Kp
        self.integral_gain = Ki
        self.derivative_gain = Kd
        # intiailze the erorr quee to update the control signal for every recived pair of errors
        self.error_queue = deque([], maxlen=2) ## for storing last two errors
        self.error_total = 0.0


    def increment_erorr(self, error):
        # add the error with its time to the error_queue
        time = rospy.get_rostime().to_sec()
        self.error_queue.append((error, time))
        # add the error to the error total
        self.error_total = self.error_total + error

        if(len(self.error_queue) > 1):
            # when multiple errors have been recorded calculate the derivative
            error_diff = self.error_queue[1][0] - self.error_queue[0][0]
            time_diff = self.error_queue[1][1] - self.error_queue[0][1]
            error_der = error_diff / time_diff
            return [error, self.error_total, error_der]
        else:
            # for first error signal no derivative is present and the total is just the error 
            return [error, error, 0]


    def get_control_signal_from_error(self, error):
        p_error, i_error, d_error = self.increment_erorr(error)

        Kp = self.proportional_gain
        Ki = self.integral_gain
        Kd = self.derivative_gain
        control_signal = Kp * p_error + Ki * i_error + Kd * d_error
        return control_signal

