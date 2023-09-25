#!/usr/bin/env python
import cv2
import numpy as np
import rospy
from std_msgs.msg import Float32MultiArray

class ArucoIOUTracker:

    def __init__(self):
        self.current_box = np.array([])
        self.pub = pub = rospy.Publisher("bestbox", Float32MultiArray, queue_size=12)
        self.sub = rospy.Subscriber("/aruco_markers", Float32MultiArray, self.subscriber_callback)
        # Initialize the OpenCV window
        cv2.namedWindow(self.window_name, cv2.WINDOW_NORMAL)
        self.window_name = "Aruco Tracking"
        self.image = None
        self.first_run = True

    def subscriber_callback(self, msg):
        # Check if it's the first run
        if self.first_run:
            print("This is the first run of the code.")
            if len(msg.data) >= 12:
            # select a random box to track then save as self.current_box=np.array[]
                self.current_box = np.array(msg.data[:12])
                print("self.current_box:", self.current_box)
            # Set the flag to False so it won't be the first run next time
            self.first_run = False
        else:
            print("This is not the first run of the code.")
            # calculate overlap and select best box
            best_IOU = 0
            best_box = []
            # Iterates through all detected boxes from the msg
            for i in range(0, len(msg.data), 12):
                box = np.array(msg.data[i:i+12])
                IOU = self.calculate_iou(self.current_box, box)
                if IOU > best_IOU:
                    best_IOU = IOU
                    best_box = box
            # Draw the best box on the image
            self.draw_box(self.image, best_box)
            # publish bestbox
            self.pub.publish(Float32MultiArray(data=best_box))
            # Update the OpenCV window
            cv2.imshow(self.window_name, self.image)
            cv2.waitKey(1)  # Update the OpenCV window
            # update current box
            self.current_box = best_box

    def draw_box(self, image, box):
        # Extract the box coordinates
        x1, y1, x2, y2 = map(int, box)

        # Draw the bounding box on the image
        cv2.rectangle(image, (x1, y1), (x2, y2), (0, 255, 0), 2)
    
    def __del__(self):
        # Destroy the OpenCV window when the node is shut down
        cv2.destroyWindow(self.window_name)

    def calculate_iou(self, box1, box2):
        # Box format: [x1, y1, x2, y2] top-left and bottom-right corners
        # Extract the coordinates of the top-left and bottom-right corners for each box
        # need to fix naming convention
        x1_tl, y1_tl, x1_br, y1_br = box1[[0 , 1 , 3 , 4]]
        x2_tl, y2_tl, x2_br, y2_br = box2[[0 , 1 , 3 , 4]]
        # Calculate the intersection area
        x1_intersection = max(x1_tl, x1_tl)
        y1_intersection = max(y1_tl, y1_tl)
        x2_intersection = min(x2_tl, x2_br)
        y2_intersection = min(y2_tl, y2_br)

        if x1_intersection < x2_intersection and y1_intersection < y2_intersection:
            intersection_area = (x2_intersection - x1_intersection) * (y2_intersection - y1_intersection)
        else:
            intersection_area = 0.0

        # Calculate the areas of both boxes
        area_box1 = (x1_br - x1_tl) * (y1_br - y1_tl)
        area_box2 = (x2_br - x2_tl) * (y2_br - y2_tl)

        # Calculate the IoU
        iou = intersection_area / float(area_box1 + area_box2 - intersection_area)

        return iou

if __name__ == "__main__":
    rospy.init_node("ArucoIOUTracker")
    ArucoIOUTracker()
    rospy.spin()  # Keep the node alive
