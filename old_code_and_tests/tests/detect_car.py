from object_detection import yolo_car_detector
from object_detection import plot_bounding_box_on_im
import cv2
import matplotlib.pyplot as plt
import glob

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


