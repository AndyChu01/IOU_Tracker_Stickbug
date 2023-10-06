import numpy as np
import rospy
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
            print("IOU Score is:",IOU)
            if IOU > best_IOU:
                best_IOU = IOU
                best_box = box
        # Publish the best box and IOU score
        best_msg = Float32MultiArray()
        best_msg.data = list(best_box) + [best_IOU]  # Append the IOU score to the best box data
        self.pub.publish(best_msg)
        # Update current box
        self.current_box = best_box
            
    def calculate_iou(self, box1, box2):
        # Note the edge case of overlapping corners were ignored
        # Box format: [xtl, ytl, xbr, ybr] top-left and bottom-right corners
        # Extract the coordinates of the top-left and bottom-right corners for each box
        # box 1
        xtl1=box1(0)
        ytl1=box1(1)
        xbr1=box1(3)
        ybr1=box1(4)
        # box 2
        xtl2=box2(0)
        ytl2=box2(1)
        xbr2=box2(3)
        ybr2=box2(4)
        # Calculate the area of the boxes
        area1 = (xbr1-xtl1)*(ybr1-ytl1)
        area2 = (xbr2-xtl2)*(ybr2-ytl2)
        # Calculate the intersections
        intx = max(0,min(xbr1,xbr2)-max(xtl1,xtl2))
        inty = max(0,min(ybr1,ybr2)-max(ytl1,ytl2))
        intArea = intx*inty
        # Calculate IOU Score
        iou = intArea/ float(area1 + area2 - intArea)
        return iou

if __name__ == "__main__":
    rospy.init_node("ArucoIOUTracker")
    ArucoIOUTracker()
    rospy.spin()  # Keep the node alive