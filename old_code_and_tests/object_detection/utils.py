import matplotlib.pyplot as plt


def plot_bounding_box_on_im(bounding_box, image):
    """
    Plot a bounding box on an image.
    :param bounding_box: The bounding box to plot, in xyxy format.
    :param image: The image to plot on.
    :return: None
    """
    plt.imshow(image)
    plt.gca().add_patch(plt.Rectangle((bounding_box[0], bounding_box[1]), bounding_box[2] - bounding_box[0],
                                      bounding_box[3] - bounding_box[1], fill=False, edgecolor='r', linewidth=2))
    # add circle to the center of the bounding box:
    plt.plot((bounding_box[0] + bounding_box[2]) / 2, (bounding_box[1] + bounding_box[3]) / 2, 'ro')
    plt.show()
