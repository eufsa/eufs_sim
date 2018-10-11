#!/usr/bin/python

# Note that twistToAckermannDrive needs to be mapped to the robot_control/command parameter in the command line ie. rosrun eufs_robot_control twist_to_ackermannDrive.py twistToAckermannDrive:=eufs_robot_control/command
# Publishes to twistToAckermannDrive
# Subscribes to cmd_vel

import rospy
import math
from ackermann_msgs.msg import AckermannDrive
from ackermann_msgs.msg import AckermannDriveStamped
from geometry_msgs.msg import Twist

class Convert:
    def __init__(self):
        self.publisher = rospy.Publisher('/robot_control/command', AckermannDriveStamped, queue_size=10)
        self.max_steering = 1
        self.min_steering = -1
        self.epsilon_steering = math.radians(0.001)

    def callback(self, data):
        ack_cmd = AckermannDriveStamped()
        ack_cmd.header.stamp = rospy.Time.now()

        drive = AckermannDrive()
        drive.speed = data.linear.x
        drive.steering_angle = data.angular.z

        # impose limits on commanded angle
        if drive.steering_angle > self.max_steering:
            drive.steering_angle = self.max_steering
        if drive.steering_angle < self.min_steering:
            drive.steering_angle = self.min_steering

        # clean up angle if it is very close to zero
        if math.fabs(drive.steering_angle) < self.epsilon_steering:
            drive.steering_angle = 0.0


        ack_cmd.drive = drive
        self.publisher.publish(ack_cmd)

    def listener(self):
        rospy.Subscriber("/cmd_vel", Twist, self.callback)
        rospy.spin()

if __name__ == '__main__':
    try:
        rospy.init_node("twistToAckermannDriveNode", anonymous=True)
        cnv = Convert()
        cnv.listener()
    except rospy.ROSInterruptException: pass
