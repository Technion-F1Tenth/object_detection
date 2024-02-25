#!/usr/bin/env python

import os
import sys
import time
import logging
import argparse

import pyrealsense2 as rs
import numpy as np
import cv2

from YOLO_car_detector import yolo_car_detector
from utils import plot_bounding_box_on_im
import matplotlib.pyplot as plt

class ObjectDetection:
    def __init__(self):
        parser = argparse.ArgumentParser(description='A simple script with command-line arguments.')

        # Define command-line arguments
        parser.add_argument('-debug', action='store_true', help='Enable debug mode')

        # Parse the command-line arguments
        args = parser.parse_args()

        # Access the parsed arguments
        debug_mode = args.debug

        # Configure logging
        logs_dir = os.getcwd() + "/logs/"
        if not os.path.exists(logs_dir):
            os.makedirs(logs_dir)

        timestamp = time.time()
        formatted_time = time.strftime("%d-%m-%Y_%H-%M-%S", time.localtime(timestamp))
        log_name = "ObjectDetection_"+ formatted_time +".log"
        log_file = logs_dir + log_name
        printColor("the log file is: " + log_file, "\033[91m")
        
        logging.basicConfig(
            level=logging.DEBUG if debug_mode else logging.INFO,  # Set the logging level
            format='%(asctime)s - %(levelname)s - %(message)s',  # Set the log format
            filename=log_file,  # Specify the log file
            filemode='w'  # Set the file mode ('w' for write, 'a' for append)
        )
    
    def detect(self, detector, images, depth_frame):
        bounding_boxes, output_images = detector.detect(images, return_images=True)
        for output_im, boxes in zip(output_images, bounding_boxes):
            cv2.namedWindow('detect', cv2.WINDOW_AUTOSIZE)
            cv2.imshow('detect', output_im)
            if boxes is not None:
                # logging.info('type:', type(boxes))
                logging.info('boxes: '+ str(boxes))
                # logging.info('shape', boxes.shape)
                point = (boxes[0], boxes[1])
                c = point[0]
                r = point[1]
                depth_intrin = depth_frame.profile.as_video_stream_profile().intrinsics
                depth = depth_frame.get_distance(c, r)
                depth_point_in_meters_camera_coords = rs.rs2_deproject_pixel_to_point(depth_intrin, [c, r], depth)

                logging.info('depth_point_in_meters_camera_coords is:' + str(depth_point_in_meters_camera_coords))

            else:
                logging.info('boxes is None! no detections')
            cv2.waitKey(1)

    def runObjectDetection(self, buffer = 1):
        # Configure depth and color streams
        pipeline = rs.pipeline()
        config = rs.config()

        config.enable_stream(rs.stream.depth, 640, 480, rs.format.z16, 30)
        config.enable_stream(rs.stream.color, 640, 480, rs.format.bgr8, 30)

        pipeline.start(config)

        detector = yolo_car_detector()
        # images_list = []
        currtnt_buffer=0
        while True:
            # Wait for a coherent pair of frames: depth and color
            frames = pipeline.wait_for_frames()
            depth_frame = frames.get_depth_frame()
            color_frame = frames.get_color_frame()
            if not depth_frame or not color_frame:
                continue

            # Convert images to numpy arrays
            depth_image = np.asanyarray(depth_frame.get_data())
            color_image = np.asanyarray(color_frame.get_data())

            # Apply colormap on depth image (image must be converted to 8-bit per pixel first)
            depth_colormap = cv2.applyColorMap(cv2.convertScaleAbs(depth_image, alpha=0.03), cv2.COLORMAP_JET)

            # images = np.hstack((color_image, depth_colormap))
            images = color_image

            currtnt_buffer += 1

            if(currtnt_buffer == buffer):
                currtnt_buffer = 0
                self.detect(detector, images, depth_frame)


def printColor(text, color_code="\033[0m"):
    print(f"{color_code}{text}\033[0m")
    
if __name__ == '__main__':
    object_detection = ObjectDetection()
    object_detection.runObjectDetection()
