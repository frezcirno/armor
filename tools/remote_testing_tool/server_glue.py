import cv2 as cv

# `frame_provider` is an object that provides a single frame,
# which should implement the method `get_current_frame: () -> np.ndarray`
# this is the actual GLUE for the `image_sending_server.py`
# you can provide your own implementation of frame_provider.
class DemoFrameProvider:
    def __init__(self):
        self._camera = cv.VideoCapture(0)
        self.get_current_frame = lambda : self._camera.read()[1]

frame_provider = DemoFrameProvider()
