#!/usr/bin/env python

import rospy
from yolact_ros_msgs.msg import Detections, Detection
from std_msgs.msg import Float64
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import cv2
import numpy as np

def best_detection_callback(msg):
    # Get the best detected flower
    best_detection = msg.detections[0]

    # Get the bounding box coordinates
    x1, y1, x2, y2 = int(best_detection.box.x1), int(best_detection.box.y1), int(best_detection.box.x2), int(best_detection.box.y2)

    # Calculate the center of the bounding box
    center_x = int((x1 + x2) / 2)
    center_y = int((y1 + y2) / 2)

    print("Received best detected flower data:")
    print("Center coordinates (x, y):", center_x, center_y)

    # Get the depth data
    depth_image = rospy.wait_for_message('/camera/depth/image_rect_raw', Image)

    # Convert depth image to OpenCV format
    bridge = CvBridge()
    depth_data = bridge.imgmsg_to_cv2(depth_image, desired_encoding='passthrough')

    # Extract the depth value for the center pixel
    depth_value = depth_data[center_y, center_x]/1000

    # Print the depth value
    print("Depth value at the center pixel:", depth_value)

    # Publish the depth value
    depth_publisher.publish(depth_value)

if __name__ == '__main__':
    rospy.init_node('distance_calculation_node')

    # Create a publisher to publish the depth value
    depth_publisher = rospy.Publisher('/flower_depth', Float64, queue_size=1)

    # Create a subscriber to subscribe to the best detected flower data
    rospy.Subscriber('/best_detected_flower', Detections, best_detection_callback)

    rospy.spin()

