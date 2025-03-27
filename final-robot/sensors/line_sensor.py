# controllers/line_sensor.py
# Módulo para manejar el sensor QTR-8RC

import RPi.GPIO as GPIO
import time
from config import SENSOR_PINS, TIEMPO_LECTURA, UMBRAL_SENSOR

class LineSensor:
    def __init__(self):
        GPIO.setmode(GPIO.BCM)
        GPIO.setwarnings(False)

        # Configurar pines de sensores como entrada
        for pin in SENSOR_PINS:
            GPIO.setup(pin, GPIO.IN)

    def leer_sensores(self):
        """Lee los valores del sensor QTR-8RC usando RC timing"""
        valores = []

        for pin in SENSOR_PINS:
            GPIO.setup(pin, GPIO.OUT)
            GPIO.output(pin, GPIO.HIGH)
            time.sleep(0.00001)  # Cargar capacitor (10 µs)

            GPIO.setup(pin, GPIO.IN)
            inicio = time.time()
            duracion = 0

            while GPIO.input(pin) and duracion < TIEMPO_LECTURA:
                duracion = time.time() - inicio

            valores.append(duracion < UMBRAL_SENSOR)  # True si detecta línea negra

        return valores

    def calcular_posicion(self, valores_sensores):
        """Calcula la posición de la línea"""
        if not any(valores_sensores):  # Si no detecta línea
            return None

        numerador = sum(i * 1000 for i, v in enumerate(valores_sensores) if v)
        denominador = sum(1 for v in valores_sensores if v)

        return numerador / denominador if denominador > 0 else 3500  # Posición central

