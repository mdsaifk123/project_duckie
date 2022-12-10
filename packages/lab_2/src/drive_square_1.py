#!/usr/bin/env python3

import rospy
from std_msgs.msg import Float32, String
from duckietown_msgs.msg import Twist2DStamped 

class drive_Square:
    def __init__(self):

        # publishes to the output of car_cmd_switch_node 
        self.pub = rospy.Publisher("car_cmd_switch_node/cmd", Twist2DStamped, queue_size=10)
        self.my_msg = Twist2DStamped() # initialized the message
    
    # sends the velocity and angular message to the 
    def send_motor_msg(self, vel, angVel):
        self.my_msg.v = vel
        self.my_msg.omega = angVel
        self.pub.publish(self.my_msg)

if __name__ == '__main__':
    try:
        drive_sq = drive_Square()
        rospy.init_node('drive_square_1', anonymous=True)

        time_straight = 3.2 # edit straight driving time here
        time_spin = 0.25 # edit body rotating time here
        run_vel = 0.50 # velocity of straight drive
        spin_omega = 2 # angular velocity used for spinning (negative spins clockwise)

        # drive straight for n seconds, then turn. Repeat 4 times
        for i in range(4):
            drive_sq.send_motor_msg(run_vel, 0) # edit velocity and omega values here
            rospy.sleep(time_straight)
            drive_sq.send_motor_msg(0, spin_omega) # spin 45 degrees
            rospy.sleep(time_spin)
            drive_sq.send_motor_msg(0, 0) # stops all motors
    except rospy.ROSInterruptException:
            pass
