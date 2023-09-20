#!/usr/bin/env python
import cv2
import numpy as np
import rospy
from std_msgs.msg import String
from std_msgs.msg import Float32MultiArray

class ArucoIOUTracker:

    def __init__(self):
        self.current = np.array([])
        self.pub = pub = rospy.Publisher("bestbox", Float32MultiArray, queue_size=12)
        self.sub = rospy.Subscriber("/multipe_aruco", Float32MultiArray, self.subscriber_callback)
    
    def subscriber_callback(self, msg):

        # Initialize a flag
        first_run = True
        # Check if it's the first run
        if first_run:
            print("This is the first run of the code.")
            # select a random box to track
            # then save as self.current_box=np.array[]
            # Set the flag to False so it won't be the first run next time
            first_run = False
        else:
            print("This is not the first run of the code.")
            # calculate overlap and select best box
            # update current box
            # publish best box

if __name__ == "__main__":
    rospy.init_node("ArucoIOUTracker")
    ArucoIOUTracker()
    rospy.spin()  # Keep the node alive