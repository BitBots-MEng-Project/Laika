import numpy
from PIL import Image


def parse_image(image: Image) -> numpy.ndarray:
    """
    Takes an opened image and returns 2D black and white representation of it.
    :param image: An image, expected to be black and white, but it will be converted internally if it's different.
    :return: NumPy 2D array representing the image, where black is 1, white is 0.
    """
    image = image.convert('L')  # Convert to monochrome
    image_array = numpy.array(image)
    image_array = ~image_array  # Flip, so white becomes 0
    image_array[image_array > 0] = 1
    return image_array
