import cv2
import numpy as np
from cv2 import aruco
import rospy
from geometry_msgs.msg import PoseStamped

# Load the camera matrix and distortion coefficients from YAML file
fs = cv2.FileStorage("camera_calibration.yml", cv2.FILE_STORAGE_READ)
camera_matrix = fs.getNode("camera_matrix").mat()
dist_coeffs = fs.getNode("distortion_coefficients").mat()
fs.release()

# Define the Aruco dictionary
aruco_dict = aruco.Dictionary_get(aruco.DICT_6X6_100)
parameters = aruco.DetectorParameters_create()

# Initialize the ROS node and publisher
rospy.init_node('aruco_pose_estimation')
pub = rospy.Publisher('object_pose', PoseStamped, queue_size=1)

# Open the webcam
cap = cv2.VideoCapture(0)

while not rospy.is_shutdown():
    # Capture a frame from the webcam
    ret, frame = cap.read()
    
    if not ret:
        rospy.logerr("Error: Failed to capture a frame from the webcam.")
        continue

    # Detect the Aruco marker
    corners, ids, rejectedImgPoints = aruco.detectMarkers(frame, aruco_dict, parameters=parameters)

    if ids is not None:
        # Estimate the pose of the Aruco marker
        rvecs, tvecs, _ = aruco.estimatePoseSingleMarkers(corners, 0.05, camera_matrix, dist_coeffs)

        # Publish the pose of the Aruco marker to the ROS topic
        pose_msg = PoseStamped()
        pose_msg.header.stamp = rospy.Time.now()
        pose_msg.header.frame_id = 'camera'
        pose_msg.pose.position.x = tvecs[0][0][0]
        pose_msg.pose.position.y = tvecs[0][0][1]
        pose_msg.pose.position.z = tvecs[0][0][2]
        pose_msg.pose.orientation.x = rvecs[0][0][0]
        pose_msg.pose.orientation.y = rvecs[0][0][1]
        pose_msg.pose.orientation.z = rvecs[0][0][2]
        pose_msg.pose.orientation.w = 1.0
        pub.publish(pose_msg)
        
        # Add the translation and rotation vectors to the video feed
        tvec_text = "Translation vector: ({:.2f}, {:.2f}, {:.2f})".format(tvecs[0][0][0], tvecs[0][0][1], tvecs[0][0][2])
        rvec_text = "Rotation vector: ({:.2f}, {:.2f}, {:.2f})".format(rvecs[0][0][0], rvecs[0][0][1], rvecs[0][0][2])
        cv2.putText(frame, tvec_text, (10, 30), cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 0, 255), 2)
        cv2.putText(frame, rvec_text, (10, 60), cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 0, 255), 2)

        # Visualize the pose of the Aruco marker
        frame = cv2.drawFrameAxes(frame, camera_matrix, dist_coeffs, rvecs, tvecs, 0.1)
        print("Translation vector:")
        print(tvecs)
        print("Rotation vector:")
        print(rvecs)

    # Display the result
    cv2.imshow('Aruco Marker', frame)
    
    # Exit if the 'q' key is pressed
    if cv2.waitKey(1) & 0xFF == ord('q'):
        break

# Release the webcam
cap.release()

# Close all windows
cv2.destroyAllWindows()
