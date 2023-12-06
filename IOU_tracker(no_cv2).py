import numpy as np
import rospy
import cv2
from std_msgs.msg import Float32MultiArray
from sensor_msgs.msg import Image
from cv_bridge import CvBridge

class YoloV8IOUTracker:
    def __init__(self):
        self.current_box = np.array([])
        self.current_marker_id=0
        self.pub = rospy.Publisher("bestbox", Float32MultiArray, queue_size=12)
        self.sub = rospy.Subscriber("/yolov8_objects", Float32MultiArray, self.subscriber_callback)
        self.imgsub = rospy.Subscriber("/camera/color/image_raw", Image, self.image_callback)
        self.first_run = True
        self.cv_image = []
        self.areavec = []
    def image_callback(self,msg):
        try:
            # Convert ROS image message to OpenCV image
            bridge = CvBridge()
            self.cv_image = bridge.imgmsg_to_cv2(msg, "bgr8")
        except Exception as e:
            print(e)
    
    def subscriber_callback(self, msg):
        current_time = rospy.get_time()
        # Check if there are ArUco markers
        max_area = 0
        idx_max=0
        array_length = len(msg.data)
        print(msg.data)
        if (array_length > 0) and self.first_run == True:
        # Check if there are at least 5 elements in each array before accessing indices 2 and 3
            for i in range(0, len(msg.data), 5):
                if len(msg.data) >= i + 5:
                    width = msg.data[i + 2] - msg.data[i]
                    height = msg.data[i + 1] - msg.data[i + 3]
                    area = np.abs(width * height)
                    self.areavec.append(area)
                    # print("AREA:", area)   
                    if area > max_area:
                        max_area = area
                        idx_max = i
                        # print("AREA:", area)
                else:
                    # Handle the case where the indices are out of range for the current array
                    print(f"Error: Indices 2 and 3 are out of range in msg.data for array {i}")
            self.areavec.append(idx_max)

            if array_length >= idx_max + 5:
                self.current_box = np.array(msg.data[idx_max:idx_max + 5])
                self.current_marker_id = int(msg.data[idx_max])
                print("self.current_box:", self.current_box)
                print("Marker ID:", self.current_marker_id)
                # Set the flag to False so it won't be the first run next time
                self.first_run = False
            else:
                print("Error: Invalid index or insufficient data for tracking.")
    
        else :
            best_IOU = 0
            best_box = []
            best_marker_id = 0
            # Iterates through all detected boxes from the msg
            for i in range(0, len(msg.data), 5):
                box = np.array(msg.data[i:i+5])
                IOU = self.calculate_iou(self.current_box, box)
                print("IOU Score is:",IOU)
                if IOU > best_IOU:
                    best_IOU = IOU
                    best_box = box
                    best_marker_id = int(msg.data[i+4])

            print("AREAS:" , self.areavec)

            # Draw bounding boxes on the image
            # image = self.draw_boxes_on_image(self.current_box, best_box)
            # cv2.imshow("YoloV8IOUTracker", image)
            # cv2.waitKey(1)

            # Publish the best box and IOU score
            best_msg = Float32MultiArray()
            best_msg.data = np.append(best_box, [best_IOU,best_marker_id]) # Append the IOU score to the best box data
            self.pub.publish(best_msg)
            # Update current box
            self.current_box = best_box
            self.current_marker_id = best_marker_id
            print(best_IOU)
            
    def calculate_iou(self, box1, box2):
        # Note the edge case of overlapping corners were ignored
        # Box format: [xtl, ytl, xbr, ybr] top-left and bottom-right corners
        # Extract the coordinates of the top-left and bottom-right corners for each box
        # box 1
        xtl1=box1[0]
        ytl1=box1[1]
        xbr1=box1[2]
        ybr1=box1[3]
        # box 2
        xtl2=box2[0]
        ytl2=box2[1]
        xbr2=box2[2]
        ybr2=box2[3]
        # calculate the area of the boxes
        area1 = max(0,xbr1-xtl1)*max(0,ybr1-ytl1)
        area2 = max(0,xbr2-xtl2)*max(0,ybr2-ytl2)
        print("area of box 1:",area1)
        print("area of box 2:",area2)
        # calculate the intersections
        intx = max(0,min(xbr1,xbr2)-max(xtl1,xtl2))
        inty = max(0,min(ybr1,ybr2)-max(ytl1,ytl2))
        intArea = intx*inty
        print("min(xbr1,xbr2):",min(xbr1,xbr2))
        print("max(xtl1,xtl2):",max(xtl1,xtl2))
        print("min(xbr1,xbr2)-max(xtl1,xtl2):",min(xbr1,xbr2)-max(xtl1,xtl2))
        print("area of intersection:",intArea)
        # calculate IOU Score
        try:
            iou = intArea/ float(area1 + area2 - intArea)
        except ZeroDivisionError:
            iou = 0.0
        return iou

    def draw_boxes_on_image(self, current_box, best_box):

            # Draw current box in blue
            cv2.rectangle(self.cv_image, (int(current_box[0]), int(current_box[1])),
                        (int(current_box[2]), int(current_box[3])), (255, 0, 0), 2)

            # Draw best box in green
            cv2.rectangle(self.cv_image, (int(best_box[0]), int(best_box[1])),
                        (int(best_box[2]), int(best_box[3])), (0, 255, 0), 2)

            return image

if __name__ == "__main__":
    rospy.init_node("YoloV8IOUTracker")
    
    tracker = YoloV8IOUTracker()

    while not rospy.is_shutdown():
        rospy.spin()

    cv2.destroyAllWindows()
