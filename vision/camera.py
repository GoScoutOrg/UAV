import cv2
from picamera2 import Picamera2
import math
# from picamera2 import Picamera2

class Arducam():
    
    def __init__(self) -> None:
        self.cam = Picamera2()
        self.cam.configure(self.cam.create_preview_configuration(main={"format": 'RGB888', "size": (640, 480)}))
        self.resolution = (640, 480)
        self.aspect_ratio = 480 / 640
        self.diagonal_fov = math.atan(self.aspect_ratio) * 2
        self.cam.start()
        
    def fetch_frame(self):
        return self.cam.capture_array()
    

# For testing, uses laptop/default webcam
class Webcam():
    
    def __init__(self) -> None:
        self.cam = cv2.VideoCapture(0)
        
    def fetch_frame(self):
        _, image = self.cam.read()
        return image