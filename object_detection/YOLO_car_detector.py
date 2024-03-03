from ultralytics import YOLO
import torch
import numpy as np




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
        #results = self.model(images, classes=self.classes_to_detect_ids, conf=self.conf_threshold,
        #                     iou=self.iou_threshold_nms, agnostic_nms=self.agnostic_nms)
        results = self.model.predict(images, conf=self.conf_threshold,
                             iou=self.iou_threshold_nms, agnostic_nms=self.agnostic_nms)
        # print('results[0].boxes.data',results[0].boxes.data)
        # print('type results[0].boxes.data',type(results[0].boxes.data))
        # print('results type', type(results))
        # print(self.model.names)

        # for each image, if there are multiple detections, take the one with the highest confidence. If there are no
        # detections, return None for that image.
        bounding_boxes = []
        for res in results:
            # print('res.boxes.data',res.boxes.data)
            # print('res shape', res.shape)
            vehicle_boxes = [detection for detection in res.boxes.data if detection[5] in self.classes_to_detect_ids]
            if not vehicle_boxes:
                bounding_boxes.append(None)
                continue
            vehicle_boxes = torch.stack(vehicle_boxes)
            # result_boxes = res.boxes.data[self.classes_to_detect_ids]
            print('vehicle_boxes.shape', vehicle_boxes.shape)
            print('vehicle_boxes',vehicle_boxes)
            # print('type vehicle_boxes[0]', type(vehicle_boxes[0]))
            # print('type vehicle_boxes', type(vehicle_boxes))
            # print(res.boxes.cls)
            # result_boxes = res.boxes
            # print('type(result_boxes)', type(result_boxes))
            # boxes = result_boxes.xyxy
            # print("class:", res._keys)
            # print("probs:", res.probs)
            # print("res.boxes.data:", res.boxes.data)

            # if boxes.shape[0] == 1:
            #     bounding_boxes.append(boxes[0].cpu().numpy())
            # elif boxes.shape[0] > 1:
            #     maximal_confidecne_det = torch.argmax(res.boxes.conf)
            #     bounding_boxes.append(boxes[maximal_confidecne_det].cpu().numpy())
            # else:
            #     bounding_boxes.append(None)

            ## passing the whole data struct: [x1 x2 y1 y2 conf label]
            if vehicle_boxes.shape[0] == 1:
                bounding_boxes.append(vehicle_boxes[0].cpu().numpy())
                print("chose the one in ", vehicle_boxes[0][0], vehicle_boxes[0][1],"with label", self.model.names[int(vehicle_boxes[0][5])])
                # print('vehicle_boxes[maximal_confidence_det].shape', vehicle_boxes[maximal_confidence_det].shape)
            elif vehicle_boxes.shape[0] > 1: # if more than one object - take highest conf
                maximal_confidence_det = torch.argmax(vehicle_boxes[:,4]) # 4 is conf!
                bounding_boxes.append(vehicle_boxes[maximal_confidence_det].cpu().numpy())
                print("chose the one in ", vehicle_boxes[maximal_confidence_det][0], vehicle_boxes[maximal_confidence_det][1])
                # print('vehicle_boxes[maximal_confidence_det].shape', vehicle_boxes[maximal_confidence_det].shape)
                print("with label", self.model.names[int(vehicle_boxes[maximal_confidence_det][5])])
            else:
                bounding_boxes.append(None)

            # if len(vehicle_boxes) == 1:
            #     bounding_boxes.append(boxes[0].cpu().numpy())
            # elif len(vehicle_boxes) > 1:
            #     maximal_confidecne_det = torch.argmax(res.boxes.conf)
            #     bounding_boxes.append(boxes[maximal_confidecne_det].cpu().numpy())
            # else:
            #     bounding_boxes.append(None)

        if not return_images:

            return bounding_boxes

        # for debug
        detection_ims = []
        for res in results:
            im_array = res.plot()
            # BGR TO RGB:
            im_array = im_array[..., ::-1]
            detection_ims.append(im_array)

        return bounding_boxes, detection_ims



