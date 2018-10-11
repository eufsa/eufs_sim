#!/usr/bin/env python

import cv2
import numpy as np
import rospy
from sensor_msgs.msg import CompressedImage
from ackermann_msgs.msg import AckermannDriveStamped
from cv_bridge import CvBridge, CvBridgeError

class TextOverlay:
    def __init__(self, title, position, text=0.0, font=cv2.FONT_HERSHEY_SIMPLEX, fontScale=1, fontColor=(0,0,255), lineType=2):
        self.title = title
        self.text = text
        self.position = position
        self.font = font
        self.fontScale = fontScale
        self.fontColor = fontColor
        self.lineType = lineType

		# call openCV text overlay method with the class variables
    def text_overlay(self, img):
        full_text = "{}: {}".format(self.title, self.text)
        cv2.putText(img,
                    full_text,
                    self.position,
                    self.font,
                    self.fontScale,
                    self.fontColor,
                    self.lineType)

    def update_text(self, text):
        self.text = text

class ImageOverlay:
    def __init__(self):
        self.velocity = TextOverlay("Velocity", (50, 50))
        self.steering = TextOverlay("Steering", (50, 100))
        self.img = np.ones((700,1000,3),dtype=np.uint8)*128 # default black background

    # update the image to the new image from camera feed
    def image_callback(self, data):
        self.img = CvBridge().compressed_imgmsg_to_cv2(data)

		# configure and overlay stats on the image
    def stats_callback(self, data):
        new_vel = data.drive.speed
        new_steer = data.drive.steering_angle
        self.velocity.update_text(new_vel)
        self.steering.update_text(new_steer)

    def overlay(self):
        img_out = self.img.copy()
        self.velocity.text_overlay(img_out)
        self.steering.text_overlay(img_out)
        #cv2.imshow('image', self.img)
        #cv2.waitKey(0)
        #cv2.destroyAllWindows()
        return img_out

    def run(self):
        rospy.init_node("overlay_image")

        self.publisher = rospy.Publisher('overlay/compressed', CompressedImage, queue_size=1)
        
        # Read in camera feed, change first arg to read a different camera
        self.camera_subscriber = rospy.Subscriber('/zed/left/image_raw/compressed', CompressedImage, self.image_callback, queue_size=1)
        # Read in stats, change this command to read in a different source of stats to overlay
        self.stats_subscriber = rospy.Subscriber('/eufs_robot_control/command', AckermannDriveStamped, self.stats_callback, queue_size=1)

        rate = rospy.Rate(3)  # default 3 Hz

        print("Running camera overlay in overlay/compressed")

        while not rospy.is_shutdown():
            img = self.overlay()
            compressed_img = CvBridge().cv2_to_compressed_imgmsg(img)
            self.publisher.publish(compressed_img)
            rate.sleep()

if __name__ == "__main__":
    img = ImageOverlay()
    img.run()
