#!/usr/bin/env python

import os
import time
import logging
import argparse

import pyrealsense2 as rs
import numpy as np
import cv2

from object_detection.yoloWorldCarDetector import yoloWorldCarDetector as yolo_car_detector

class ObjectDetection:
    def __init__(self):
        parser = argparse.ArgumentParser(description='A simple script with command-line arguments.')

        # Define command-line arguments
        parser.add_argument('--debug', action='store_true', help='Enable debug mode')

        # Parse the command-line arguments
        args = parser.parse_args()

        # Access the parsed arguments
        self.debug_mode = args.debug

        # Configure logging
        logs_dir = os.getcwd() + "/logs/"
        if not os.path.exists(logs_dir):
            os.makedirs(logs_dir)

        timestamp = time.time()
        formatted_time = time.strftime("%d-%m-%Y_%H-%M-%S", time.localtime(timestamp))
        log_name = "ObjectDetection_"+ formatted_time +".log"
        log_file = logs_dir + log_name
        printColor("the log file is: " + log_file, "\033[95m")
        
        logging.basicConfig(
            level=logging.DEBUG if self.debug_mode else logging.INFO,  # Set the logging level
            format='%(asctime)s - %(levelname)s - %(message)s',  # Set the log format
            filename=log_file,  # Specify the log file
            filemode='w'  # Set the file mode ('w' for write, 'a' for append)
        )
        logging.info("!!!!!!!START!!!!!!!")


        # init realsense camera pipeline
        self.pipeline = rs.pipeline()
        config = rs.config()

        config.enable_stream(rs.stream.depth, 640, 480, rs.format.z16, 30)
        config.enable_stream(rs.stream.color, 640, 480, rs.format.bgr8, 30)

        self.pipeline.start(config)

        # initialize camera consts
        frame = self.pipeline.wait_for_frames()
        depth_frame = frame.get_depth_frame()
        self.depth_intrin = depth_frame.profile.as_video_stream_profile().intrinsics

        self.detector = yolo_car_detector()

        fourcc = cv2.VideoWriter_fourcc(*'mp4v')

        # Define the video resolution based on the first image
        clean_video = logs_dir + "ObjectDetection_" + formatted_time + "_clean.mp4"
        data_video = logs_dir + "ObjectDetection_" + formatted_time + "_withData.mp4"
        self.clean_video = cv2.VideoWriter(clean_video, fourcc, 10, (640, 480))
        self.data_video = cv2.VideoWriter(data_video, fourcc, 10, (640, 480))


    def get_opponent_xy_point(self, box):
        if box is not None:
            top_left_point = (box[0], box[1])
            bottom_right_point = (box[2], box[3])

            # center of the car
            c = int((top_left_point[0] + bottom_right_point[0]) / 2)
            r = int((top_left_point[1] + bottom_right_point[1]) / 2)

            return (c, r)
        
    def runObjectDetection(self, buffer = 1):
        while True:
            # Wait for a coherent pair of frames: depth and color
            frames = self.pipeline.wait_for_frames()
            depth_frame = frames.get_depth_frame()
            color_frame = frames.get_color_frame()
            if not depth_frame or not color_frame:
                continue

            # Convert images to numpy arrays
            color_image = np.asanyarray(color_frame.get_data())

            bounding_box, output_images = self.detector.detect(color_image, return_images=True)
            cv2.namedWindow('detect', cv2.WINDOW_AUTOSIZE)
            cv2.imshow('detect', output_images)
            cv2.waitKey(1)

            # Write images to the video
            self.clean_video.write(color_image)

        

            logging.debug(bounding_box)

            if bounding_box is not None:
                c, r = self.get_opponent_xy_point(bounding_box)

                depth = depth_frame.get_distance(c, r)
                depth_point_in_meters_camera_coords = rs.rs2_deproject_pixel_to_point(self.depth_intrin, [c, r], depth)

                cv2.rectangle(output_images, (x1, y1), (x2, y2), (255, 0, 0), 2)  # Blue color bbox with 2px thickness
                self.data_video.write(output_images)

                logging.info('depth_point_in_meters_camera_coords is: ' + str(depth_point_in_meters_camera_coords))
            else:
                logging.info('boxes is None! no detections')
        # Release the video writer
        self.clean_video.release()
        self.data_video.release()
            


def printColor(text, color_code="\033[0m"):
    print(f"{color_code}{text}\033[0m")

def main():
    object_detection = ObjectDetection()
    object_detection.runObjectDetection()

if __name__ == '__main__':
    main()





