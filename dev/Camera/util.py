import cv2
import numpy as np

def loadImage(image):
    """Loads the image.

    - If `image` is `str`, it will be interpreted as the image's URL.
    - Otherwise, `image` is returned as-is
    
    Raise:
    ---------
    Raises an error if `image` is `str` and the image cannot be loaded from the URL."""
    if isinstance(image, str):
        image = cv2.imread(image)     

    return image
