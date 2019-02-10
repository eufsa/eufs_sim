#!/usr/bin/env python

"""ground_truth_republisher.py

Republishes a corrected version of the raw ground truth state estimate from the simulation in the
same frame as the state estimator.
"""

import numpy as np
import math
import rospy
import tf
from nav_msgs.msg import Odometry

class GroundTruthRepublisher(object):
  
  def __init__(self, namespace='ground_truth_republisher'):
    """Initialize this _ground_truth_republisher"""
    rospy.init_node("ground_truth_republisher", anonymous = True)
    self.pub = rospy.Publisher("/ground_truth/state", Odometry, queue_size = 1)
    self.sub = rospy.Subscriber("/ground_truth/state_raw", Odometry, self.handle_pose)
  
  def handle_pose(self, msg):
    #set frame to be the same as state estimator output
    msg.header.frame_id='map'
    msg.child_frame_id='base_footprint'
    self.pub.publish(msg)

if __name__ == "__main__":
  ground_truth_republisher = GroundTruthRepublisher()
  rospy.spin()
