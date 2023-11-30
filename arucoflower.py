#!/usr/bin/env python

import rospy
import cv2
import cv2.aruco as aruco
from cv_bridge import CvBridge
from sensor_msgs.msg import Image, CameraInfo
#from geometry_msgs.msg import Point
from sensor_msgs.msg import PointCloud2
import sensor_msgs.point_cloud2 as pc2
import numpy as np
from std_msgs.msg import Float32MultiArray

class ArucoDetectionNode:
    def __init__(self):
        rospy.init_node('aruco_detection_node')

        self.bridge = CvBridge()
        self.image_sub = rospy.Subscriber('/camera/color/image_raw', Image, self.image_callback)
        self.depth_sub = rospy.Subscriber('/camera/depth/image_rect_raw', Image, self.depth_callback)
        self.aruco_pub = rospy.Publisher('/aruco_markers', Float32MultiArray, queue_size=10)

        self.aruco_dict = aruco.Dictionary_get(aruco.DICT_6X6_250)
        self.parameters = aruco.DetectorParameters_create()

        self.window_name = 'Aruco Detection'
        cv2.namedWindow(self.window_name)

        self.depth_image = None

    def depth_callback(self, msg):
        try:
            cv_depth = self.bridge.imgmsg_to_cv2(msg, desired_encoding="passthrough")
            self.depth_image = np.array(cv_depth, dtype=np.float32) 
        except CvBridgeError as e:
            rospy.logerr(e)

    def image_callback(self, msg):
        try:
            cv_image = self.bridge.imgmsg_to_cv2(msg, "bgr8")
        except CvBridgeError as e:
            rospy.logerr(e)
            return

        gray = cv2.cvtColor(cv_image, cv2.COLOR_BGR2GRAY)
        corners, ids, _ = aruco.detectMarkers(gray, self.aruco_dict, parameters=self.parameters)
        # Draw ArUco marker on the image
        cv2.aruco.drawDetectedMarkers(cv_image, corners, ids)

        if ids is not None and self.depth_image is not None:
            # Publish marker depth information
            aruco_info = Float32MultiArray()

            for i in range(len(ids)):
                marker_id = ids[i][0]
                marker_corners = corners[i][0]

                # Calculate the average depth value for the marker corners
                corner_depths = []
                for corner in marker_corners:
                    x, y = int(corner[0]), int(corner[1])
                    if 0 <= x < self.depth_image.shape[1] and 0 <= y < self.depth_image.shape[0]:
                        depth = self.depth_image[y, x]/1000
                        corner_depths.append(depth)

                if len(corner_depths) > 0:
                    # Calculate the average depth
                    average_depth = np.mean(corner_depths)

                    aruco_info.data.append(marker_corners[0][0]) # X-coordinate of top-left corner
                    aruco_info.data.append(marker_corners[0][1])  # Y-coordinate of top-left corner
                    aruco_info.data.append(average_depth)

                    aruco_info.data.append(marker_corners[1][0])  # X-coordinate of top-right corner
                    aruco_info.data.append(marker_corners[1][1])  # Y-coordinate of top-right corner
                    aruco_info.data.append(average_depth)
                    
                    aruco_info.data.append(marker_corners[2][0]) # X-coordinate of bottom-right corner
                    aruco_info.data.append(marker_corners[2][1])  # Y-coordinate of bottom-right corner
                    aruco_info.data.append(average_depth)
                    
                    aruco_info.data.append(marker_corners[3][0])  # X-coordinate of bottom-left corner
                    aruco_info.data.append(marker_corners[3][1])  # Y-coordinate of bottom-left corner
                    aruco_info.data.append(average_depth)



            #aruco_info.z = average_depth
            self.aruco_pub.publish(aruco_info)
        # Display the annotated image
        cv2.imshow(self.window_name, cv_image)
        cv2.waitKey(1)

def main():
    aruco_detection_node = ArucoDetectionNode()
    rospy.spin()

if __name__ == '__main__':
    main()

