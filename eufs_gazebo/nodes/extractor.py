#!/usr/bin/env python

""" Cone extractor from Gazebo simulation in the frame of the car

Subscribed Topics:
    Odometry (nav_msgs/Odometry)
        Position and velocities of the robot

Published Topics:
    coneArray (eufs_msgs.msg/coneArray)
        Cone locations in the frame of the car
    MarkerArray (visualization_msgs/MarkerArray)
        Cone locations to be displayed in Rviz
    
Parameters:
    ~view_distance (float, default: 30)
        Only the cones that are within this distance are published 
    ~pose_topic (string, default: /ground_truth/state_raw)
        Topic name for Odometry subscriber
    ~track_path (string, default: eufs_gazebo/tracks/small_track.csv)
        Path to the track csv file
    ~cone_topic (string, default: /cones_ground_truth)
        Topic name for coneArray publisher
    ~marker_topic (string, default: map_extract_path/cone_markers)
        Topic name for MarkerArray publisher

Todo:
    -fix odometry misalignment


The MIT License

Copyright (c) 2018-2018 Edinburgh University Formula Student (EUFS)
http://eufs.eusa.ed.ac.uk

Permission is hereby granted, free of charge, to any person obtaining a copy
of this software and associated documentation files (the "Software"), to deal
in the Software without restriction, including without limitation the rights
to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
copies of the Software, and to permit persons to whom the Software is
furnished to do so, subject to the following conditions:

The above copyright notice and this permission notice shall be included in
all copies or substantial portions of the Software.

THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN
THE SOFTWARE.
"""

import pandas as pd
import numpy as np
from numpy import matlib
from eufs_msgs.msg import coneArray
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Point
from visualization_msgs.msg import Marker, MarkerArray
import tf
import rospy

class Extractor:

    """The Extractor class
    """    
    def __init__(self):
        rospy.init_node("cone_ground_truth")
        self.left_cones     = None
        self.right_cones    = None
        self.big_cones      = None
        self.distance       = rospy.get_param("~view_distance", default=30.)
        self.tf_listener    = tf.TransformListener()
        self.BASE_FOOTPRINT = "/base_footprint"
        self.MAP            = "/map"


        track_path = rospy.get_param("~track_path", default="eufs_gazebo/tracks/small_track.csv")
        self.load_csv(track_path)

        pose_topic = rospy.get_param("~pose_topic", default="/ground_truth/state_raw")
        self.subscriber = rospy.Subscriber(pose_topic, Odometry, self.odom_cb)

        cone_topic = rospy.get_param("~cone_topic", default="/cones_ground_truth")
        self.cone_pub = rospy.Publisher(cone_topic, coneArray, queue_size=1)

        marker_topic = rospy.get_param("~marker_topic", default="map_extract_path/cone_markers")
        self.cone_marker_pub = rospy.Publisher(marker_topic, MarkerArray, queue_size=1)

    def mul_by_transpose(self, X, size):
        """Helper function for self.dists().
        Multiplies each vector of a matrix by its transpose.

        Args:
            X (np.array): matrix whose vectors to multiply
            size (int): size of the matrix
        Returns:
            Matrix of vectors multiplied by their transposes.
        """
        result = []
        for i in range(size):
            result.append(np.dot(X[i], X[i].T))
        return np.expand_dims(np.array(result).T, axis=1)

    def dists(self, Xtrn, Xtst):
        """Calculates distances from one matrix vectors to 
            the other.

        Args:
            Xtrn (np.array): first matrix
            Xtrn (np.array): second matrix matrix

        Returns:
            Matrix of distances from the first matrix vectors to the second.
        """
        [M, _] = np.shape(Xtrn)
        [N, _] = np.shape(Xtst)
        mul = np.dot(Xtst, Xtrn.T)

        XX = self.mul_by_transpose(Xtst, N)
        YY = self.mul_by_transpose(Xtrn, M)

        return np.add(np.subtract(matlib.repmat(XX, 1, M), 2*mul), (np.matlib.repmat(YY, 1, N)).T)
        
    def process_cones(self, cones_list, yaw, trans):
        """Loads a Gazebo model .SDF file and identify cones in it
        based on their mesh tag. Store as elements of the class.

        Args:
            file_path (str): the path to the SDF file to load

        Returns:
            Nothing
        """
        transform = np.array([[trans[0], trans[1]]])
        cones_dists = self.dists(np.array(cones_list), transform)

        closest_cones = []
        for i in range(len(cones_dists[0])):
            if cones_dists[0][i] < np.power(self.distance, 2):
                closest_cones.append(cones_list[i])

        rotation_matrix = np.array([[ np.cos(yaw),  np.sin(yaw)],\
                                    [-np.sin(yaw),  np.cos(yaw)]])

        
        translation = closest_cones - np.repeat(np.array(transform), np.shape(closest_cones)[0], axis=0)        
        cones_rotated = np.dot(rotation_matrix, translation.T).T
               
        cones_euler = []
        for cone in cones_rotated:
            #Filter cones that are in front of the car
            if cone[0] > 0:
                cones_euler.append([cone[0], cone[1]])       
        return cones_euler

    def odom_cb(self, msg):
        """Callback function that translates the cone locations
            and publishes them.

        Args:
            msg (str): subsriber message

        Returns:
            Nothing
        """

        #The translation fails at the beggining, so we want to ignore these exceptions 
        try:
            (trans, rot) = self.tf_listener.lookupTransform(self.MAP, self.BASE_FOOTPRINT, rospy.Time(0))
            yaw = tf.transformations.euler_from_quaternion(rot)[2]
            
            left_closest_cones = self.process_cones(self.left_cones, yaw, trans)
            right_closest_cones = self.process_cones(self.right_cones, yaw, trans)

            cone_msg = coneArray()
            cone_markers = MarkerArray()
            marker_id = 0
            for cone in np.reshape(left_closest_cones, (-1, 2)):
                cone_msg.left_cones.append(Point(cone[0], cone[1], 0))
                marker = self.get_cone_marker(False, self.BASE_FOOTPRINT, cone[0], cone[1], 0, marker_id)
                marker_id += 1
                cone_markers.markers.append(marker)
                cone_markers.markers.append(marker)

            for cone in np.reshape(right_closest_cones, (-1, 2)):
                cone_msg.right_cones.append(Point(cone[0], cone[1], 0))
                marker = self.get_cone_marker(True, self.BASE_FOOTPRINT, cone[0], cone[1], 0, marker_id)
                marker_id += 1
                cone_markers.markers.append(marker) 

            self.cone_marker_pub.publish(cone_markers)
            self.cone_pub.publish(cone_msg)

        except (tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException):
            print("Got exception")

    def get_cone_marker(self, is_inner, frame, x, y, z, id):
        """Prepares a Marker for publishing

        Args:
            is_inner (boolean): flag to distinguish inner from outer cones of the track
            frame (str): frame in which the position is given
            x (str): x position of the cone
            y (str): y position of the cone
            z (str): z position of the cone
            id (str): distinct id of the marker

        Returns:
            Marker ready for publishing
        """
        m = Marker()
        m.header.frame_id = frame
        m.id = id
        m.header.stamp = rospy.get_rostime()
        m.type = Marker.MESH_RESOURCE
        m.action = Marker.ADD
        m.pose.position.x = x
        m.pose.position.y = y
        m.pose.position.z = z
        m.pose.orientation.x = 0.0
        m.pose.orientation.y = 0.0
        m.pose.orientation.z = 0.0
        m.pose.orientation.w = 1.0
        m.scale.x = 1.5
        m.scale.y = 1.5
        m.scale.z = 1.5
        m.mesh_resource = "package://eufs_description/meshes/cone.dae"
        m.color.a = 1.0
        # inner are yellow
        if is_inner:
            m.color.r = 1.0
            m.color.g = 1.0
            m.color.b = 0.0
        else:
            m.color.r = 0.0
            m.color.g = 0.0
            m.color.b = 1.0

        return m  

    def load_csv(self, file_path):
        """Loads CSV file of cone location data and store it in the class.
        CSV file must have a 1 line header and then each row should have 
        the format tag, x, y where tag is either "big", "left" or "right"

        Args:
            file_path (str): the path to the CSV file to load

        Returns:
            Nothing
        """
        
        if file_path.find(".csv") == -1:
            print("Please give me a .csv file. Exitting")
            return
        
        data = pd.read_csv(file_path, names=["tag", "x", "y"], skiprows=1)
        self.left_cones = np.array(data[data.tag == "left"][["x", "y"]])
        self.right_cones = np.array(data[data.tag == "right"][["x", "y"]])
        self.big_cones = np.array(data[data.tag == "big"][["x", "y"]])
        
    def load_sdf(self, file_path):
        """Loads a Gazebo model .SDF file and identify cones in it
        based on their mesh tag. Store as elements of the class.

        Args:
            file_path (str): the path to the SDF file to load

        Returns:
            Nothing
        """
        
        if file_path.find(".sdf") == -1:
            print("Please give me a .sdf file. Exitting")
            return
        
        root = ET.parse(file_path).getroot()
        left = []
        right = []
        big = []
        
        # iterate over all links of the model
        if len(root[0].findall("link")) != 0:
            for child in root[0].iter("link"):
                pose = child.find("pose").text.split(" ")[0:2]
                mesh_str = child.find("visual")[0][0][0].text.split("/")[-1]

                # indentify cones by the name of their mesh
                if "blue" in mesh_str:
                    left.append(pose)
                elif "yellow" in mesh_str:
                    right.append(pose)
                elif "big" in mesh_str:
                    big.append(pose)
            
        elif len(root[0].findall("include")) != 0:
            for child in root[0].iter("include"):
                pose = child.find("pose").text.split(" ")[0:2]
                mesh_str = child.find("uri").text.split("/")[-1]

                # indentify cones by the name of their mesh
                if "blue" in mesh_str:
                    left.append(pose)
                elif "yellow" in mesh_str:
                    right.append(pose)
                elif "big" in mesh_str:
                    big.append(pose)
            

                
        # convert all lists to numpy as arrays for efficiency
        self.left_cones = np.array(left, dtype="float64")
        self.right_cones = np.array(right, dtype="float64")
        if len(big) != 0:
            self.big_cones = np.array(big, dtype="float64")

if __name__=="__main__":
    extractor = Extractor()
    rospy.spin()


    