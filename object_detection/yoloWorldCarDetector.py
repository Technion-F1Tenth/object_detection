# from inference.models.yolo_world.yolo_world import YOLOWorld
# import supervision as sv
from ultralytics import YOLOWorld
import cv2
import glob
import time
import torch
import logging
import sys
import numpy as np



class yoloWorldCarDetector:
    def __init__(self, confidence=0.1, iou_threshold=0.6, agnostic_nms=False, debug=False):
        """
        :param conf_threshold: The confidence threshold for the model to consider a detection as valid.
        :param iou_threshold_nms: The iou between two bounding boxes for the non-maximum suppression to consider
            them as the same object.
        :param agnostic_nms: If True, the non-maximum suppression will be performed between all classes
        """
        self.confidence = confidence 
        self.agnostic_nms = agnostic_nms
        self.iou_threshold = iou_threshold

        self.model = YOLOWorld('yolov8s-world.pt')

        self.classes = ['toy car', 'toy car tire', 'toy car wheel', 
                        # 'aluminium hose', 'shiny blockade'
                        'autonomic toy car computation unit', ]
        self.model.set_classes(self.classes)
        self._debug = debug

    def detect(self, image, return_images=False):
        """
        get bounding coodinates for the cars in the images as xyxy format.
        :param image:
        :param return_images: If True, the function will return the images with the bounding boxes drawn on them. Those
            images will contain all detections, including the ones with low confidence that were filtered out.
        :return: list of xyxy bounding boxes, in the length of the input images.
        """
        t = time.time()
        results = self.model.predict(image, conf=0.1, iou=self.iou_threshold, agnostic_nms=False) #TODO shuold we use iou threshold? we later take the one with highest conf anyway...


        logging.debug(f"Time for inference: {time.time() - t:.4f} seconds")
        result = results[0]  # we only support one image at a time

        confidences = result.boxes.conf
        if len(confidences) == 0:
            return None if not return_images else None, image

        best_detection = torch.argmax(confidences)
        bounding_box = result.boxes.xyxy[best_detection]

        if not return_images:
            return bounding_box
        else:
            annotated_image = result.plot()
            return bounding_box, annotated_image

