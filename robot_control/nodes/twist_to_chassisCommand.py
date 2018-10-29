#!/usr/bin/python

# Note that twistToAckermannDrive needs to be mapped to the robot_control/command parameter in the command line ie. rosrun eufs_robot_control twist_to_ackermannDrive.py twistToAckermannDrive:=eufs_robot_control/command
# Publishes to twistToAckermannDrive
# Subscribes to cmd_vel

import rospy
import math
from autorally_msgs.msg import chassisCommand
from autorally_msgs.msg import runstop
from geometry_msgs.msg import Twist
import sys, select, termios, tty
import threading

class Convert:
    def __init__(self):
        self.settings = termios.tcgetattr(sys.stdin)
        self.publisher = rospy.Publisher('/joystick/chassisCommand', chassisCommand, queue_size=10)
        self.publisher_runstop = rospy.Publisher('/runstop', runstop, queue_size=10)
        self.max_steering = 1.
        self.min_steering = -1.
        self.max_throttle = 1.
        self.min_throttle = -1.
        self.epsilon_steering = math.radians(0.001)
        self.runstop = False

    def callback(self, data):
        cmd = chassisCommand()
        cmd.header.stamp = rospy.Time.now()
        cmd.sender = "rqt"
        cmd.throttle = data.linear.x
        cmd.steering = data.angular.z
        cmd.frontBrake = 0.

        # impose limits on commanded angle
        if cmd.steering > self.max_steering:
            cmd.steering = self.max_steering
        if cmd.steering < self.min_steering:
            cmd.steering = self.min_steering

        # impose limits on steering
        if cmd.throttle > self.max_throttle:
            cmd.throttle = self.max_throttle
        if cmd.throttle < self.min_throttle:
            cmd.throttle = self.min_throttle

        # clean up angle if it is very close to zero
        if math.fabs(cmd.steering) < self.epsilon_steering:
            cmd.steering = 0.0

        self.publisher.publish(cmd)

    def listener(self):
        rospy.Subscriber("/rqt/cmd_vel", Twist, self.callback)
        rospy.spin()

    def getKey(self):
        tty.setraw(sys.stdin.fileno())
        select.select([sys.stdin], [], [], 0)
        key = sys.stdin.read(1)
        termios.tcsetattr(sys.stdin, termios.TCSADRAIN, self.settings)
        return key

    def dealKey(self):
        while True:
            input_cmd = self.getKey()
            if input_cmd == "\x03":  # ctrl-c key
                break
            else:
                self.runstop = not self.runstop
                print("Toggling runstop to", self.runstop, input_cmd)

                run_cmd = runstop()
                run_cmd.header.stamp = rospy.Time.now()
                run_cmd.sender = "rqt"
                run_cmd.motionEnabled = self.runstop

                self.publisher_runstop.publish(run_cmd)

if __name__ == '__main__':
    try:
        rospy.init_node("twistToChassisCommand", anonymous=True)
        print("You can toggle runstop with any key and enter")
        cnv = Convert()
        threading.Thread(target=cnv.dealKey).start()
        cnv.listener()

    except rospy.ROSInterruptException: pass
   
