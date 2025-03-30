#!/usr/bin/env python3
# -*- coding: utf-8 -*-

"""
Script para detección y conteo de objetos rojos/verdes en tiempo real con Raspberry Pi + OpenCV.
Autor: Ingeniera Electrónica
Fecha: [Fecha]
"""

import cv2
import numpy as np

# ===================== CONFIGURACIÓN AJUSTABLE =====================
# Parámetros para detección de color (espacio HSV)
COLOR_RANGES = {
    "red": {
        "lower1": [0, 100, 100],    # Rojo (rango bajo)
        "upper1": [10, 255, 255],
        "lower2": [160, 100, 100],   # Rojo (rango alto)
        "upper2": [180, 255, 255]
    },
    "green": {
        "lower": [40, 50, 50],      # Verde
        "upper": [90, 255, 255]
    }
}

# Parámetros de filtrado de contornos
MIN_AREA = 500           # Área mínima para considerar un objeto (píxeles)
BLUR_SIZE = (5, 5)       # Tamaño del kernel para desenfoque gaussiano

# Configuración de visualización
DISPLAY_TEXT_COLOR = {
    "red": (0, 0, 255),  # BGR: Rojo
    "green": (0, 255, 0) # BGR: Verde
}
TEXT_POSITION = {
    "red": (10, 30),     # Posición (x, y) del texto para rojos
    "green": (10, 70)    # Posición (x, y) del texto para verdes
}
FONT = cv2.FONT_HERSHEY_SIMPLEX
FONT_SCALE = 1
FONT_THICKNESS = 2

# ===================== FUNCIONES PRINCIPALES =====================
def apply_color_mask(hsv_frame, color_config):
    """Aplica máscaras de color para un rango específico."""
    if color_config.get("lower2"):  # Caso especial para rojo (2 rangos)
        lower1 = np.array(color_config["lower1"])
        upper1 = np.array(color_config["upper1"])
        lower2 = np.array(color_config["lower2"])
        upper2 = np.array(color_config["upper2"])
        mask1 = cv2.inRange(hsv_frame, lower1, upper1)
        mask2 = cv2.inRange(hsv_frame, lower2, upper2)
        return mask1 + mask2
    else:
        lower = np.array(color_config["lower"])
        upper = np.array(color_config["upper"])
        return cv2.inRange(hsv_frame, lower, upper)

def count_objects(mask, min_area):
    """Cuenta objetos válidos en una máscara binaria."""
    contours, _ = cv2.findContours(mask, cv2.RETR_TREE, cv2.CHAIN_APPROX_SIMPLE)
    return len([cnt for cnt in contours if cv2.contourArea(cnt) > min_area])

def main():
    # Inicializar cámara
    cap = cv2.VideoCapture(0)
    if not cap.isOpened():
        print("Error: No se pudo acceder a la cámara.")
        return

    try:
        while True:
            ret, frame = cap.read()
            if not ret:
                print("Error: Frame no válido.")
                break

            # Preprocesamiento
            blurred = cv2.GaussianBlur(frame, BLUR_SIZE, 0)
            hsv = cv2.cvtColor(blurred, cv2.COLOR_BGR2HSV)

            # Detección de colores
            mask_red = apply_color_mask(hsv, COLOR_RANGES["red"])
            mask_green = apply_color_mask(hsv, COLOR_RANGES["green"])

            # Conteo de objetos
            red_count = count_objects(mask_red, MIN_AREA)
            green_count = count_objects(mask_green, MIN_AREA)

            # Visualización
            cv2.putText(frame, f"ROJOS: {red_count}", 
                        TEXT_POSITION["red"], FONT, FONT_SCALE, 
                        DISPLAY_TEXT_COLOR["red"], FONT_THICKNESS)
            cv2.putText(frame, f"VERDES: {green_count}", 
                        TEXT_POSITION["green"], FONT, FONT_SCALE, 
                        DISPLAY_TEXT_COLOR["green"], FONT_THICKNESS)

            cv2.imshow("Deteccion de Objetos", frame)

            # Salir con 'q'
            if cv2.waitKey(1) & 0xFF == ord('q'):
                break

    finally:
        cap.release()
        cv2.destroyAllWindows()

if __name__ == "__main__":
    main()
