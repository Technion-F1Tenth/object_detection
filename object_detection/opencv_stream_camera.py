
import sys

import pyrealsense2 as rs
import numpy as np
import cv2

from YOLO_car_detector import yolo_car_detector
from utils import plot_bounding_box_on_im
import matplotlib.pyplot as plt

# buffer = 5 #TODO: costomaize config 

def main(buffer = 1):
    # Configure depth and color streams
    pipeline = rs.pipeline()
    config = rs.config()

    config.enable_stream(rs.stream.depth, 640, 480, rs.format.z16, 30)
    config.enable_stream(rs.stream.color, 640, 480, rs.format.bgr8, 30)

    pipeline.start(config)

    detector = yolo_car_detector()
    # images_list = []
    i=0
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

        # # Show images
        # cv2.namedWindow('RealSense', cv2.WINDOW_AUTOSIZE)
        # cv2.imshow('RealSense', images)
        # cv2.waitKey(1)

        #detect
        # images_list.appand(images)
        i+=1
        if(i==buffer):
            i=0

            bounding_boxes, output_images = detector.detect(images, return_images=True)
            for output_im, boxes in zip(output_images, bounding_boxes):
                cv2.namedWindow('detect', cv2.WINDOW_AUTOSIZE)
                cv2.imshow('detect', output_im)
                if boxes is not None:
                    # print('type:', type(boxes))
                    print('boxes:', boxes)
                    # print('shape', boxes.shape)
                    point = (boxes[0], boxes[1])
                    c = point[0]
                    r = point[1]
                    depth_intrin = depth_frame.profile.as_video_stream_profile().intrinsics
                    depth = depth_frame.get_distance(c, r)
                    depth_point_in_meters_camera_coords = rs.rs2_deproject_pixel_to_point(depth_intrin, [c, r], depth)
                    xyz = [depth_point_in_meters_camera_coords[2], -depth_point_in_meters_camera_coords[0], -depth_point_in_meters_camera_coords[1]]
                    print('depth_point_in_meters_camera_coords is:' ,depth_point_in_meters_camera_coords)
                    print('x:', xyz[0], 'y:', xyz[1], "z:", xyz[2])
                else:
                    print ('boxes is None! no detections')
                cv2.waitKey(1)
            # images_list = []


    pipeline.stop() #TODO: never get here




if __name__ == '__main__':
    args_num = len(sys.argv) - 1
    if not args_num: main()
    elif args_num == 1: main(sys.argv[1])
    else: print("Too many arguments!")

