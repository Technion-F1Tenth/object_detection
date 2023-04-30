import cv2
import numpy as np
from cv2 import aruco

# Load the camera matrix and distortion coefficients from YAML file
fs = cv2.FileStorage("camera_calibration.yml", cv2.FILE_STORAGE_READ)
camera_matrix = fs.getNode("camera_matrix").mat()
dist_coeffs = fs.getNode("dist_coeff").mat()
fs.release()
print(type(camera_matrix))
print(dist_coeffs)

# Define the Aruco dictionary
aruco_dict = aruco.Dictionary_get(aruco.DICT_6X6_100)
parameters = aruco.DetectorParameters_create()

# Open the webcam
cap = cv2.VideoCapture(0)

while True:
    # Capture a frame from the webcam
    ret, frame = cap.read()
    
    if not ret:
        print("Error: Failed to capture a frame from the webcam.")
        break

    # Detect the Aruco marker
    corners, ids, rejectedImgPoints = aruco.detectMarkers(frame, aruco_dict, parameters=parameters)

    if ids is not None:
        # Estimate the pose of the Aruco marker
        rvecs, tvecs, _ = aruco.estimatePoseSingleMarkers(corners, 0.05, camera_matrix, dist_coeffs)
        
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
