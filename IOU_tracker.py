#!/usr/bin/env python
import cv2
import numpy as np
import rospy
from std_msgs.msg import String
from std_msgs.msg import Float32MultiArray

class ArucoIOUTracker:
    
    # determine if 1st run
    # select a random box to track
    # then save as self.current_box=np.array[]
    # if not first run
    # calculate overlap and select best box
    # update current box

    
    def subscriber_callback(data):
        # Check if the data length is a multiple of 12
        if len(data.data) % 12 == 0:
            n = len(data.data) // 12  # Calculate the value of N
            rospy.loginfo("Received a 1x%d Float32MultiArray:", n)
        
            # Extract and process the data in groups of 12
            for i in range(n):
                start_idx = i * 12
                end_idx = (i + 1) * 12
                subarray = data.data[start_idx:end_idx]
                rospy.loginfo("Subarray %d: %s", i + 1, subarray)
        else:
            rospy.logwarn("Received data does not have a valid length (not a multiple of 12). Ignoring.")

    def main():
        rospy.init_node("float32_multiarray_subscriber")
    
        # Subscribe to the topic publishing the Float32MultiArray
        rospy.Subscriber("/multipe_aruco", Float32MultiArray, subscriber_callback)
        
        # Publish to a topic
        pub = rospy.Publisher("output_topic", String, queue_size=10)
        message= 
        pub.publish(message)

        rospy.spin()  # Keep the node alive

    if __name__ == "__main__":
        main()
