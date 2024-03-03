from inference.models.yolo_world.yolo_world import YOLOWorld
import cv2
import supervision as sv
import glob
import time


images_paths = glob.glob("../data/sample_other_car_images/color_image*.png")

images = [cv2.imread(image_path) for image_path in images_paths]

model = YOLOWorld(model_id="yolo_world/s")

classes = ['toy car', 'tire', 'toy car tire', 'toy car wheel', 'ventilation hose',
           'autonomic toy car computation unit', ]
model.set_classes(classes)

for image in images:
    t = time.time()

    results = model.infer(image, confidence=0.1, agnostic_nms=False)
    detections = sv.Detections.from_inference(results)

    print(f"Time for inference: {time.time() - t:.4f} seconds")

    labels = [
        f"{classes[class_id]} {confidence:0.3f}"
        for class_id, confidence
        in zip(detections.class_id, detections.confidence)
    ]


    annotated_image = image.copy()
    annotated_image = sv.BoundingBoxAnnotator().annotate(annotated_image, detections)
    annotated_image = sv.LabelAnnotator().annotate(annotated_image, detections, labels=labels)
    sv.plot_image(annotated_image)
