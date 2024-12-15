# from object_detection import yolo_car_detector
# from object_detection import plot_bounding_box_on_im
import sys, os, time

from yoloWorldCarDetector import yoloWorldCarDetector as yolo_car_detector

import numpy as np
import cv2
import matplotlib.pyplot as plt
import glob


def detectCarV1():
    images = glob.glob('../data/sample_other_car_images/color_image*.png')

    detector = yolo_car_detector()
    bounding_boxes, output_images = detector.detect(images, return_images=True)

    for output_im, boxes, im_path in zip(output_images, bounding_boxes, images):
        plt.imshow(output_im)
        plt.show()
        print(boxes)

        image = cv2.imread(im_path)
        image = cv2.cvtColor(image, cv2.COLOR_BGR2RGB)
        plot_bounding_box_on_im(boxes, image)
        plt.show()


# not support depth yet
def detectCarV2(source_video_path, dest_video_path=None, logs_dir=os.path.dirname(__file__) + "/logs/", depth_radius=5):
    timestamp = time.localtime()
    formatted_time = time.strftime("%d-%m-%Y_%H-%M-%S", timestamp)
    log_name = "ObjectDetection_" + formatted_time + ".log"
    # prep results
    fourcc = cv2.VideoWriter_fourcc(*'mp4v')

    # Define the video resolution based on the first image
    clean_video_path = logs_dir + "ObjectDetection_" + formatted_time + "_clean_drive_ with_big.mp4"
    data_video_path = logs_dir + "ObjectDetection_" + formatted_time + "_withData_drive_with_big.mp4"

    clean_video = cv2.VideoWriter(clean_video_path, fourcc, 10, (640, 480))
    data_video = cv2.VideoWriter(data_video_path, fourcc, 10, (640, 480))

    detector = yolo_car_detector()
    source_video_capture = cv2.VideoCapture(source_video_path)
    depth_video_capture = cv2.VideoCapture(dest_video_path) if dest_video_path else None

    while True:
        ret, color_frame = source_video_capture.read()
        if not ret: break

        if dest_video_path:
            ret, depth_frame = depth_video_capture.read()
            if not ret: break
        else:
            depth_frame = None

        # color_image = np.asanyarray(color_frame.get_data())
        bounding_box, output_images = detector.detect(color_frame, return_images=True)
        print(bounding_box)
        clean_video.write(color_frame)

        if bounding_box is not None:
            # c, r = get_opponent_xy_point(bounding_box)

            # depth = depth_frame.get_distance(c, r)
            # depth_point_in_meters_camera_coords = rs.rs2_deproject_pixel_to_point(depth_intrin, [c, r], depth)
            # if dest_video_path:
            #     depth_point_in_meters_camera_coords, c, r = getDepth(depth_frame, bounding_box)
            #     depth_point_in_meters_camera_coords *= 3  # TODO: factor
            #     print('depth_point_in_meters_camera_coords is:' + str(depth_point_in_meters_camera_coords))
            #
            #     cv2.rectangle(output_images, (c - depth_radius, r - depth_radius),
            #                   (c + depth_radius, r + depth_radius), (255, 0, 0),
            #                   2)  # Blue color bbox with 2px thickness
            #     cv2.putText(output_images, "distance" + str(depth_point_in_meters_camera_coords), (20, 20),
            #                 cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255, 0, 0), 1)
            # cv2.putText(output_images, "camera:" + str(rs.camera_info.name), (20, 50), cv2.FONT_HERSHEY_SIMPLEX, 0.5,
            #             (255, 255, 0), 1)

            # cv2.rectangle(output_images, (c - depth_radius, r - depth_radius),(c + depth_radius, r + depth_radius), (255, 0, 0), 2)  # Blue color bbox with 2px thickness
            print("Write images to the data video")
            data_video.write(output_images)


        else:
            print('boxes is None! no detections')
            data_video.write(color_frame)


if __name__ == "__main__":
    source_video_path = "/Users/toamelharar/Downloads/test_videos_29_03_2024/ObjectDetection_21-11-2023_21-38-02_clean.mp4"
    logs_dir = "/Users/toamelharar/Documents/GitHub/object_detection/object_detection/object_detection/source_for_README" + "/logs/"
    if not os.path.exists(logs_dir):
        os.makedirs(logs_dir)
    print(f"log directory is: {logs_dir}")

    detectCarV2(source_video_path = source_video_path, logs_dir = logs_dir)