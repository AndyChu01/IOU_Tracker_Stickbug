
import rospy
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import cv2
from ultralytics import YOLO
import pyrealsense2 as rs
import numpy as np
from ultralytics.utils.plotting import Annotator
from std_msgs.msg import Float32MultiArray




class flower_tracker_class:
    def __init__(self):
        self.yolo_pub = rospy.Publisher('/yolov8_objects',Float32MultiArray,queue_size=10)
        self.imgsub = rospy.Subscriber("/camera/color/image_raw", Image, self.image_callback)
        self.boxsub = rospy.Subscriber('/bestbox', Float32MultiArray, self.best_box_callback)
        self.best_box_data = []
        self.cv_image = []
        self.model = YOLO('best_bramble.pt')
        self.iou_score = 0

    def best_box_callback(self,data):
        # global best_box_data
        self.best_box_data = data.data
        self.iou_score = self.best_box_data[5]
        print("Box co-ordinates in callback \n")
        print(self.best_box_data[0])
        print(self.best_box_data[3])

    def image_callback(self,msg):
        try:
            # Convert ROS image message to OpenCV image
            bridge = CvBridge()
            self.cv_image = bridge.imgmsg_to_cv2(msg, "bgr8")

            results = self.model.track(self.cv_image,persist=True,show=True)
            # print("results")
            # print(len(results))
            annotated_frame = results[0].plot()

            # yolov8 object list
            yolo_info_list = []
            
            for r in results:
                annotator = Annotator(self.cv_image)
                boxes = r.boxes
                ids = r.boxes.id
                #print ("the id value is ",ids)
                #print("the box values are ",boxes)
                for box in boxes:
                    b = box.xyxy[0]  # get box coordinates in (top, left, bottom, right) format              
                    id=box[0].id
                    #print("the id is ",id)
                    #print("the box value is ", b)
                    #print ("flower id %d has box %d"%(id,b))
                    #c = box.cls
                    #print("class is ",c)
                    #id=
                    #annotator.box_label(b, model.names[int(c)])
                    yolo_info_list.extend([
                        b[0], b[1],
                        b[2], b[3],
                        id
                    ])
            
            # Display the camera feed
            #cv2.imshow("Camera Feed", color_image)
            # Publish yolo_info as Float32MultiArray msg
            yolo_info = Float32MultiArray(data=yolo_info_list)
            self.yolo_pub.publish(yolo_info)
            print("Type of best box data\n")
            print(type(self.best_box_data))
            if self.best_box_data : 
                print("BEST BOX DATA : \n")
                print(self.best_box_data)
                cv2.rectangle(self.cv_image, (int(self.best_box_data[0]), int(self.best_box_data[1])),
                        (int(self.best_box_data[2]), int(self.best_box_data[3])), (0, 255, 0), 2)
                # Display the IOU score on the annotated frame
                iou_score_text = f"IOU Score: {self.iou_score}"  # Replace 'iou_score' with your actual IOU score
                cv2.putText(self.cv_image, iou_score_text, (10, 30), cv2.FONT_HERSHEY_SIMPLEX, 1, (0, 255, 0), 2)
            # Display the annotated frame
            cv2.imshow("Tracking Flowers", self.cv_image)
            
            # Break the loop when 'q' is pressed
            # if cv2.waitKey(1) & 0xFF == ord('q'):
            #    break

        except Exception as e:
            print(e)
    

if __name__ == "__main__":
    rospy.init_node("flowerTracker")
    
    tracker = flower_tracker_class()

    while not rospy.is_shutdown():
        rospy.spin()

    cv2.destroyAllWindows()