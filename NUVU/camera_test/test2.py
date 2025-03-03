import RPi.GPIO as GPIO
import time
import cv2
import numpy as np
from picamera2 import Picamera2

class RobotControl:
    def __init__(self):
        # ============= CONFIGURACIONES =============
        # Configuración GPIO
        GPIO.setmode(GPIO.BCM)
        GPIO.setwarnings(False)

        # Pines de motores
        self.MOTOR_PINS = {
            'ENA': 26,
            'IN1': 19,
            'IN2': 13,
            'ENB': 11,
            'IN3': 6,
            'IN4': 5
        }

        # Configuración cámara
        self.CAMERA_CONFIG = {
            'format': 'XRGB8888',
            'size': (640, 480)  # Ancho x Alto
        }

        # Rangos de color HSV
        self.COLOR_RANGES = {
            'green': {'lower': [35, 50, 50], 'upper': [85, 255, 255]},
            'red': {
                'lower1': [0, 50, 50], 'upper1': [10, 255, 255],
                'lower2': [170, 50, 50], 'upper2': [180, 255, 255]
            }
        }

        # Parámetros de control
        self.VELOCIDADES = {
            'adelante': 60,    # Velocidad cuando avanza
            'atras': -40,      # Velocidad cuando retrocede
            'giro': 40,        # Velocidad de giro para seguimiento
            'busqueda': 30     # Velocidad de giro en búsqueda
        }

        self.UMBRALES = {
            'min_area': 1000,        # Área mínima para considerar detección
            'umbral_centrado': 50,   # Margen para considerar el objeto centrado (píxeles)
            'ancho_frame': 640       # Ancho del frame de la cámara
        }
        # ============================================

        # Inicialización de componentes
        self.setup_motores()
        self.setup_camara()

    def setup_motores(self):
        # Configurar pines de motores
        for pin in self.MOTOR_PINS.values():
            GPIO.setup(pin, GPIO.OUT)
            GPIO.output(pin, GPIO.LOW)

        # Configurar PWM
        self.pwm_a = GPIO.PWM(self.MOTOR_PINS['ENA'], 1000)
        self.pwm_b = GPIO.PWM(self.MOTOR_PINS['ENB'], 1000)
        self.pwm_a.start(0)
        self.pwm_b.start(0)

    def setup_camara(self):
        self.picam2 = Picamera2()
        self.picam2.configure(self.picam2.create_preview_configuration(
            main=self.CAMERA_CONFIG))
        self.picam2.start()

    def control_motores(self, izq, der):
        # Limitar velocidades
        izq = max(min(izq, 100), -100)
        der = max(min(der, 100), -100)

        # Control motor izquierdo
        GPIO.output(self.MOTOR_PINS['IN1'], GPIO.HIGH if izq >=0 else GPIO.LOW)
        GPIO.output(self.MOTOR_PINS['IN2'], GPIO.LOW if izq >=0 else GPIO.HIGH)
        self.pwm_a.ChangeDutyCycle(abs(izq))

        # Control motor derecho
        GPIO.output(self.MOTOR_PINS['IN3'], GPIO.HIGH if der >=0 else GPIO.LOW)
        GPIO.output(self.MOTOR_PINS['IN4'], GPIO.LOW if der >=0 else GPIO.HIGH)
        self.pwm_b.ChangeDutyCycle(abs(der))

    def detectar_objetos(self):
        frame = self.picam2.capture_array()
        frame = cv2.cvtColor(frame, cv2.COLOR_RGBA2BGR)
        hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)
        
        # Detectar colores
        green_mask = cv2.inRange(hsv, 
            np.array(self.COLOR_RANGES['green']['lower']),
            np.array(self.COLOR_RANGES['green']['upper']))
        
        red_mask1 = cv2.inRange(hsv,
            np.array(self.COLOR_RANGES['red']['lower1']),
            np.array(self.COLOR_RANGES['red']['upper1']))
        red_mask2 = cv2.inRange(hsv,
            np.array(self.COLOR_RANGES['red']['lower2']),
            np.array(self.COLOR_RANGES['red']['upper2']))
        red_mask = cv2.bitwise_or(red_mask1, red_mask2)

        # Encontrar centroides
        def obtener_centroide(mask):
            contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
            if contours:
                max_contour = max(contours, key=cv2.contourArea)
                if cv2.contourArea(max_contour) > self.UMBRALES['min_area']:
                    M = cv2.moments(max_contour)
                    if M["m00"] != 0:
                        return (int(M["m10"] / M["m00"]), int(M["m01"] / M["m00"]))
            return None

        return {
            'green': obtener_centroide(green_mask),
            'red': obtener_centroide(red_mask)
        }

    def seguir_color(self, centroide):
        centro_x = self.UMBRALES['ancho_frame'] // 2
        if centroide is None:
            # Giro en búsqueda
            self.control_motores(-self.VELOCIDADES['busqueda'], self.VELOCIDADES['busqueda'])
            return False
        
        diferencia = centroide[0] - centro_x
        if abs(diferencia) < self.UMBRALES['umbral_centrado']:
            # Objeto centrado: avanzar
            self.control_motores(self.VELOCIDADES['adelante'], self.VELOCIDADES['adelante'])
            return True
        else:
            # Ajustar dirección
            if diferencia < 0:
                # Girar izquierda
                self.control_motores(-self.VELOCIDADES['giro'], self.VELOCIDADES['giro'])
            else:
                # Girar derecha
                self.control_motores(self.VELOCIDADES['giro'], -self.VELOCIDADES['giro'])
            return False

    def ejecutar(self):
        try:
            print("Iniciando sistema de seguimiento...")
            while True:
                objetos = self.detectar_objetos()
                
                # Prioridad: verde primero
                if objetos['green']:
                    if self.seguir_color(objetos['green']):
                        print("Verde detectado: Avanzando")
                    else:
                        print("Verde detectado: Ajustando dirección")
                
                elif objetos['red']:
                    print("Rojo detectado: Retrocediendo")
                    self.control_motores(self.VELOCIDADES['atras'], self.VELOCIDADES['atras'])
                
                else:
                    print("Buscando objetivo...")
                    self.control_motores(-self.VELOCIDADES['busqueda'], self.VELOCIDADES['busqueda'])

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
    robot.ejecutar()
