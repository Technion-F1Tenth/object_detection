from inference.models.yolo_world.yolo_world import YOLOWorld
import cv2
import supervision as sv
import glob
import time
import torch
import numpy as np



class yoloWorldCarDetector:
    def __init__(self, confidence=0.1, agnostic_nms=False, debug=False):
        """
        :param conf_threshold: The confidence threshold for the model to consider a detection as valid.
        :param iou_threshold_nms: The iou between two bounding boxes for the non-maximum suppression to consider
            them as the same object.
        :param agnostic_nms: If True, the non-maximum suppression will be performed between all classes
        """
        self.confidence = confidence 
        self.agnostic_nms = agnostic_nms
        self.model = YOLOWorld(model_id="yolo_world/s")

        self.classes = ['toy car', 'toy car tire', 'toy car wheel', 
                        # 'aluminium hose', 'shiny blockade'
                        'autonomic toy car computation unit', ]
        self.model.set_classes(self.classes)
        self._debug = debug

    def detect(self, image, return_images=False):
        """
        get bounding coodinates for the cars in the images as xyxy format.
        :param images: list of images or paths to images.
        :param return_images: If True, the function will return the images with the bounding boxes drawn on them. Those
            images will contain all detections, including the ones with low confidence that were filtered out.
        :return: list of xyxy bounding boxes, in the length of the input images.
        """
        t = time.time()
        results = self.model.infer(image, confidence=0.1, agnostic_nms=False) #TODO shuold we use iou threshold? we later take the one with highest conf anyway...
        detections = sv.Detections.from_inference(results)

        print(f"Time for inference: {time.time() - t:.4f} seconds")

        labels = [
            f"{self.classes[class_id]}: {confidence:0.3f}, tracker_id {tracker_id}"
            for class_id, confidence, tracker_id
            in zip(detections.class_id, detections.confidence, detections.tracker_id)
        ]

        annotated_image = image.copy()
        annotated_image = sv.BoundingBoxAnnotator().annotate(annotated_image, detections)
        annotated_image = sv.LabelAnnotator().annotate(annotated_image, detections, labels=labels)
        bounding_boxes = []
        boxes = detections.xyxy

        if boxes.shape[0] == 1:
                bounding_boxes.append(boxes[0])
        elif boxes.shape[0] > 1:
            maximal_confidecne_det = np.argmax(detections.confidence)
            bounding_boxes.append(boxes[maximal_confidecne_det])
        else:
            bounding_boxes.append(None)

        if not return_images:
            return bounding_boxes
        
        # for debug
        if self._debug:
            detection_ims = []
            for res in results:
                im_array = res.plot()
                # BGR TO RGB:
                im_array = im_array[..., ::-1]
                detection_ims.append(im_array)
        return bounding_boxes, annotated_image
    
        # return bounding_boxes, detection_ims

