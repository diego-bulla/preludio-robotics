import cv2
import numpy as np
from picamera2 import Picamera2
from config import CAMERA
from utils.gpio_manager import gpio_protected

class CameraController:
    def __init__(self):
        self.picam2 = Picamera2()
        self._configure_camera()
        
    def _configure_camera(self):
        config = self.picam2.create_preview_configuration(
            main={"size": CAMERA['resolution'], "format": CAMERA['format']})
        self.picam2.configure(config)
        
    @gpio_protected
    def start(self):
        self.picam2.start()
        
    @gpio_protected    
    def get_frame(self):
        frame = self.picam2.capture_array()
        return cv2.cvtColor(frame, cv2.COLOR_RGBA2BGR)
        
    @gpio_protected
    def stop(self):
        self.picam2.stop()
