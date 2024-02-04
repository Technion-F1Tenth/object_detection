from ultralytics import YOLO
import torch


class yolo_car_detector:
    def __init__(self, conf_threshold=0.05, iou_threshold_nms=0.3, agnostic_nms=True):
        """
        :param conf_threshold: The confidence threshold for the model to consider a detection as valid.
        :param iou_threshold_nms: The iou between two bounding boxes for the non-maximum suppression to consider
            them as the same object.
        :param agnostic_nms: If True, the non-maximum suppression will be performed between all classes
        """
        
        #TODO: check the best config
        self.conf_threshold = conf_threshold 
        self.iou_threshold_nms = iou_threshold_nms
        self.agnostic_nms = agnostic_nms

        self.model = YOLO('yolov8n.pt')

        # we only want output for these classes, which our car can be classified as:
        classes_to_detect = ['car', 'motorcycle', 'bus', 'truck', 'bicycle', 'skateboard']
        self.classes_to_detect_ids = [key for key, value in self.model.names.items() if value in classes_to_detect]

    def detect(self, images, return_images=False):
        """
        get bounding coodinates for the cars in the images as xyxy format.
        :param images: list of images or paths to images.
        :param return_images: If True, the function will return the images with the bounding boxes drawn on them. Those
            images will contain all detections, including the ones with low confidence that were filtered out.
        :return: list of xyxy bounding boxes, in the length of the input images.
        """
        results = self.model(images, classes=self.classes_to_detect_ids, conf=self.conf_threshold,
                             iou=self.iou_threshold_nms, agnostic_nms=self.agnostic_nms)

        # for each image, if there are multiple detections, take the one with the highest confidence. If there are no
        # detections, return None for that image.
        bounding_boxes = []
        for res in results:
            boxes = res.boxes.xyxy
            if boxes.shape[0] == 1:
                bounding_boxes.append(boxes[0].cpu().numpy())
            elif boxes.shape[0] > 1:
                maximal_confidecne_det = torch.argmax(res.boxes.conf)
                bounding_boxes.append(boxes[maximal_confidecne_det].cpu().numpy())
            else:
                bounding_boxes.append(None)

        if not return_images:
            return bounding_boxes

        #for debug
        detection_ims = []
        for res in results:
            im_array = res.plot()
            # BGR TO RGB:
            im_array = im_array[..., ::-1]
            detection_ims.append(im_array)

        return bounding_boxes, detection_ims



