# IOU_Tracker_Stickbug
WVU Stickbug Visual Servoing
Aruco code is for testing multiple Aruco and can be ignored

NOTE: To run the IoU and tracker
1. Put the best_bramble.pt, flower_tracker_class.py, and IOU_tracker.py in the same folder
2. roslaunch realsense2_camera rs_camera.launch
3. rviz rviz add camera image raw (Optional)
4. cd the workspace
5. source devel/setup.bash
6. In two separate terminals run:
       python3 flower_tracker_class.py
       python3 IOU_tracker.py

