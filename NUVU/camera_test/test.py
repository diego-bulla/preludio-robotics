import RPi.GPIO as GPIO
import time
import cv2
import numpy as np
from picamera2 import Picamera2

class RobotControl:
    def __init__(self):
        # Configuración GPIO
        GPIO.setmode(GPIO.BCM)
        GPIO.setwarnings(False)
        
        # Configuración de pines del motor
        self.ENA = 26
        self.IN1 = 19
        self.IN2 = 13
        self.ENB = 11
        self.IN3 = 6
        self.IN4 = 5
        
        self.setup_motores()
        
        # Configuración de la cámara
        self.picam2 = Picamera2()
        self.picam2.configure(self.picam2.create_preview_configuration(
            main={"format": 'XRGB8888', "size": (640, 480)}))
        self.picam2.start()
        
        # Rangos de color en HSV
        self.lower_green = np.array([35, 50, 50])
        self.upper_green = np.array([85, 255, 255])
        self.lower_red1 = np.array([0, 50, 50])
        self.upper_red1 = np.array([10, 255, 255])
        self.lower_red2 = np.array([170, 50, 50])
        self.upper_red2 = np.array([180, 255, 255])
        
        # Velocidades
        self.velocidad_adelante = 60
        self.velocidad_atras = -40

    def setup_motores(self):
        # Configurar pines de motores
        motor_pins = [self.ENA, self.IN1, self.IN2, self.ENB, self.IN3, self.IN4]
        for pin in motor_pins:
            GPIO.setup(pin, GPIO.OUT)
            GPIO.output(pin, GPIO.LOW)
        
        # Configurar PWM
        self.pwm_a = GPIO.PWM(self.ENA, 1000)
        self.pwm_b = GPIO.PWM(self.ENB, 1000)
        self.pwm_a.start(0)
        self.pwm_b.start(0)

    def control_motores(self, izq, der):
        # Limitar velocidades
        izq = max(min(izq, 100), -100)
        der = max(min(der, 100), -100)
        
        # Control motor izquierdo
        GPIO.output(self.IN1, GPIO.HIGH if izq >=0 else GPIO.LOW)
        GPIO.output(self.IN2, GPIO.LOW if izq >=0 else GPIO.HIGH)
        self.pwm_a.ChangeDutyCycle(abs(izq))
        
        # Control motor derecho
        GPIO.output(self.IN3, GPIO.HIGH if der >=0 else GPIO.LOW)
        GPIO.output(self.IN4, GPIO.LOW if der >=0 else GPIO.HIGH)
        self.pwm_b.ChangeDutyCycle(abs(der))

    def detectar_colores(self):
        # Capturar imagen
        frame = self.picam2.capture_array()
        frame = cv2.cvtColor(frame, cv2.COLOR_RGBA2BGR)
        hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)
        
        # Máscaras para colores
        mask_green = cv2.inRange(hsv, self.lower_green, self.upper_green)
        mask_red1 = cv2.inRange(hsv, self.lower_red1, self.upper_red1)
        mask_red2 = cv2.inRange(hsv, self.lower_red2, self.upper_red2)
        mask_red = cv2.bitwise_or(mask_red1, mask_red2)
        
        # Detección de áreas
        green_detected = cv2.countNonZero(mask_green) > 1000
        red_detected = cv2.countNonZero(mask_red) > 1000
        
        return green_detected, red_detected

    def ejecutar(self):
        try:
            while True:
                green, red = self.detectar_colores()
                
                if green:
                    print("Detectado VERDE - Avanzando")
                    self.control_motores(self.velocidad_adelante, self.velocidad_adelante)
                elif red:
                    print("Detectado ROJO - Retrocediendo")
                    self.control_motores(self.velocidad_atras, self.velocidad_atras)
                else:
                    print("No se detectan colores - Detenido")
                    self.control_motores(0, 0)
                
                time.sleep(0.1)
                
        except KeyboardInterrupt:
            self.detener()

    def detener(self):
        self.control_motores(0, 0)
        self.pwm_a.stop()
        self.pwm_b.stop()
        self.picam2.stop()
        GPIO.cleanup()
        print("\nSistema detenido correctamente")

if __name__ == "__main__":
    robot = RobotControl()
    print("Iniciando sistema de detección por color...")
    print("Presiona Ctrl+C para salir")
    robot.ejecutar()
