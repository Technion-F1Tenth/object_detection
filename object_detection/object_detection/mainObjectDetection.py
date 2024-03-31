#!/usr/bin/env python

import os
import time
import logging
import argparse

import pyrealsense2 as rs
import numpy as np
import cv2

from .yoloWorldCarDetector import yoloWorldCarDetector as yolo_car_detector

class ObjectDetection:
    def __init__(self, debug = False, depth_radius = 5, offset_x=0, offset_y=0):
        parser = argparse.ArgumentParser(description='A simple script with command-line arguments.')

        # Define command-line arguments
        parser.add_argument('--debug', action='store_true', help='Enable debug mode')
        parser.add_argument('--dr', required=False , help='change the default radius for measure depth')
        parser.add_argument('--ox', required=False , help='change the default offset_x for measure depth')
        parser.add_argument('--oy', required=False , help='change the default offset_y for measure depth')

        # Parse the command-line arguments
        args = parser.parse_args()

        # Access the parsed arguments
        self.debug_mode = args.debug or debug
        self.depth_radius = args.dr if args.dr else depth_radius 
        self.offset_x = args.ox if args.ox else offset_x 
        self.offset_y = args.oy if args.oy else offset_y 

        # Configure logging
        logs_dir = "/home/doof-wagon/f1tenth_ws/object_detection/logs_videos/"
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
        printColor(clean_video, "\033[93m")
        printColor(data_video, "\033[93m")
        
        
        
        self.clean_video = cv2.VideoWriter(clean_video, fourcc, 10, (640, 480))
        self.data_video = cv2.VideoWriter(data_video, fourcc, 10, (640, 480))

    def getOpponentXYPoint(self, box, offset_x, offset_y):
        if box is not None:
            top_left_point = (box[0], box[1])
            bottom_right_point = (box[2], box[3])
            logging.debug("BBOX: " + str(top_left_point) + ", " + str(bottom_right_point))

            # center of the car
            c = int((top_left_point[0] + bottom_right_point[0]) / 2) + offset_x
            r = int((top_left_point[1] + bottom_right_point[1]) / 2) + offset_y

            logging.debug("center point:" + str(c) + ", " + str(r))
            return c, r
        
        return None, None
    
    def getDepth(self, depth_frame, bbox):
        depth_list = []
        c,r = self.getOpponentXYPoint(bbox, self.offset_x, self.offset_y)
        for row in range(r - self.depth_radius, r + self.depth_radius):
            for col in range(c - self.depth_radius, c + self.depth_radius):
                if row > bbox[1] and row < bbox[3] and col > bbox[0] and col < bbox[2]: 
                    depth = depth_frame.get_distance(col, row)
                    depth_list.append(rs.rs2_deproject_pixel_to_point(self.depth_intrin, [col, row], depth))
        if not depth_list:
            logging.error("depth list is empty")
            return None, None, None
        return np.mean(depth_list), c, r
        
    def runObjectDetection(self, buffer = 1, ros = False):
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
            
            if not ros:
                cv2.namedWindow('detect', cv2.WINDOW_AUTOSIZE)
                cv2.imshow('detect', output_images)
                cv2.waitKey(1)

            print("Write images to the clean video")
            self.clean_video.write(color_image)


            logging.debug(bounding_box)

            if bounding_box is not None:
                # c, r = self.get_opponent_xy_point(bounding_box)

                # depth = depth_frame.get_distance(c, r)
                # depth_point_in_meters_camera_coords = rs.rs2_deproject_pixel_to_point(self.depth_intrin, [c, r], depth)
                depth_point_in_meters_camera_coords, c, r = self.getDepth(depth_frame, bounding_box)
                logging.info('depth_point_in_meters_camera_coords is:' + str(depth_point_in_meters_camera_coords))

                cv2.rectangle(output_images, (c-self.depth_radius, r-self.depth_radius), (c+self.depth_radius, r+self.depth_radius), (255, 0, 0), 2)  # Blue color bbox with 2px thickness
                cv2.putText(output_images, "distance" + str(depth_point_in_meters_camera_coords), (c-self.depth_radius-10, r-self.depth_radius-10), cv2.FONT_HERSHEY_SIMPLEX, 1.5, (255,0,0), 2) 
                print("Write images to the data video")
                self.data_video.write(output_images)
                
                if ros:
                    return [c,r,depth_point_in_meters_camera_coords], output_images
            else:
                logging.info('boxes is None! no detections')
                self.data_video.write(color_image)
                if ros:
                    return None, None
        self.clean_video.release()
        self.data_video.release()

def printColor(text, color_code="\033[0m"):
    print(f"{color_code}{text}\033[0m")

def main():
    object_detection = ObjectDetection()
    object_detection.runObjectDetection()

if __name__ == '__main__':
    main()





