import cv2
import numpy as np
from controllers.camera import Camera

class ObjectDetector:
    def __init__(self, min_area=500):
        """Inicializa el detector de objetos con un área mínima para filtrar ruido."""
        self.min_area = min_area
        self.camera = Camera()

    def detect(self):
        """Detecta objetos en la imagen basada en contornos y devuelve sus coordenadas."""
        frame = self.camera.get_frame()
        gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
        blurred = cv2.GaussianBlur(gray, (5, 5), 0)
        _, thresh = cv2.threshold(blurred, 100, 255, cv2.THRESH_BINARY)

        contours, _ = cv2.findContours(thresh, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
        objects = []

        for cnt in contours:
            area = cv2.contourArea(cnt)
            if area > self.min_area:
                x, y, w, h = cv2.boundingRect(cnt)
                objects.append((x, y, w, h))

        return objects

if __name__ == "__main__":
    detector = ObjectDetector()
    try:
        while True:
            objects = detector.detect()
            print(f"Objetos detectados: {objects}")
    except KeyboardInterrupt:
        pass
