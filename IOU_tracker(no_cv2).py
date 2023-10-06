import numpy as np
import rospy
import time
from std_msgs.msg import Float32MultiArray

class ArucoIOUTracker:
    def __init__(self):
        self.current_box = np.array([])
        self.pub = pub = rospy.Publisher("bestbox", Float32MultiArray, queue_size=12)
        self.sub = rospy.Subscriber("/aruco_markers", Float32MultiArray, self.subscriber_callback)
        self.first_run = True
    
    def subscriber_callback(self, msg):
        # Check if there are ArUco markers
        if len(msg.data) >= 12:
            # Select a random box to track then save as self.current_box=np.array[]
            self.current_box = np.array(msg.data[:12])
            print("self.current_box:", self.current_box)
            # Set the flag to False so it won't be the first run next time
            self.first_run = False
        else:
            # ArUco markers are not detected yet, continue waiting
            print("No ArUco markers detected yet. Waiting...")
            return  # Exit the callback function without executing the else block

        # At this point, ArUco markers have been detected
        # Calculate overlap and select best box
        best_IOU = 0
        best_box = []
        # Iterates through all detected boxes from the msg
        for i in range(0, len(msg.data), 12):
            box = np.array(msg.data[i:i+12])
            IOU = self.calculate_iou(self.current_box, box)
            if IOU > best_IOU:
                best_IOU = IOU
                best_box = box
        # Publish the best box and IOU score
        best_msg = Float32MultiArray()
        best_msg.data = list(best_box) + [best_IOU]  # Append the IOU score to the best box data
        self.pub.publish(best_msg)
        # Update current box
        self.current_box = best_box