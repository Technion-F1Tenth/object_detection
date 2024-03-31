import cv2

# Open the first video
cap1 = cv2.VideoCapture(r'/object_detection/logs/ObjectDetection_29-03-2024_11-36-34_clean.mp4')
# Open the second video
cap2 = cv2.VideoCapture(r'/object_detection/logs/ObjectDetection_29-03-2024_11-36-34_withData.mp4')

# Check if both videos are opened successfully
print("-DGB-", cap1.isOpened())
print("-DGB-", cap2.isOpened())

if not cap1.isOpened() or not cap2.isOpened():
    print("Error opening video streams")
    exit(1)

# Read until both videos are completed
while cap1.isOpened() and cap2.isOpened():
    ret1, frame1 = cap1.read()
    ret2, frame2 = cap2.read()

    # Break the loop if there are no frames to read
    if not ret1 or not ret2:
        break

    # Resize the frames to the same size (optional)
    frame1 = cv2.resize(frame1, (640, 480))
    frame2 = cv2.resize(frame2, (640, 480))

    # Concatenate the frames horizontally
    combined_frame = cv2.hconcat([frame1, frame2])

    # Display the concatenated frame
    cv2.imshow('Frame', combined_frame)

    # Press Q on keyboard to exit
    if cv2.waitKey(25) & 0xFF == ord('q'):
        break

# When everything done, release the video capture objects
cap1.release()
cap2.release()

# Close all the frames
cv2.destroyAllWindows()
