import cv2
import numpy as np
from config import OBJECT_DETECTION

class ObjectDetector:
    def __init__(self):
        self.kernel = np.ones((5,5), np.uint8)
        
    def detect(self, frame):
        hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)
        results = {}
        
        for color, ranges in OBJECT_DETECTION['color_ranges'].items():
            if color == 'red':
                mask1 = cv2.inRange(hsv, np.array(ranges['lower1']), np.array(ranges['upper1']))
                mask2 = cv2.inRange(hsv, np.array(ranges['lower2']), np.array(ranges['upper2']))
                mask = cv2.bitwise_or(mask1, mask2)
            else:
                mask = cv2.inRange(hsv, np.array(ranges['lower']), np.array(ranges['upper']))
                
            mask = cv2.morphologyEx(mask, cv2.MORPH_OPEN, self.kernel)
            results[color] = self._get_centroid(mask, color)
            
        return results
    
    def _get_centroid(self, mask, color):
        contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
        if not contours:
            return None
            
        max_contour = max(contours, key=cv2.contourArea)
        area = cv2.contourArea(max_contour)
        
        min_area = OBJECT_DETECTION['min_area']
        if color == 'red':
            min_area *= OBJECT_DETECTION['color_ranges']['red']['confidence']
            
        if area > min_area:
            M = cv2.moments(max_contour)
            if M["m00"] != 0:
                return (int(M["m10"] / M["m00"]), int(M["m01"] / M["m00"]), area
        return None
