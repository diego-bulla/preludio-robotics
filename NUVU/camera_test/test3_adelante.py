import RPi.GPIO as GPIO
import time
import cv2
import numpy as np
from picamera2 import Picamera2

class RobotControl:
    def __init__(self):
        # ============= CONFIGURACIONES ACTUALIZADAS =============
        # Configuración GPIO (sin cambios)
        GPIO.setmode(GPIO.BCM)
        GPIO.setwarnings(False)

        # Pines de motores (sin cambios)
        self.MOTOR_PINS = {
            'ENA': 26,
            'IN1': 19,
            'IN2': 13,
            'ENB': 11,
            'IN3': 6,
            'IN4': 5
        }

        # Configuración cámara (sin cambios)
        self.CAMERA_CONFIG = {
            'format': 'XRGB8888',
            'size': (640, 480)
        }

        # Rangos de color HSV (sin cambios)
        self.COLOR_RANGES = {
            'green': {'lower': [35, 50, 50], 'upper': [85, 255, 255]},
            'red': {
                'lower1': [0, 50, 50], 'upper1': [10, 255, 255],
                'lower2': [170, 50, 50], 'upper2': [180, 255, 255]
            }
        }

        # NUEVOS PARÁMETROS DE CONTROL PARA CÁMARA TRASERA
        self.VELOCIDADES = {
            'acercarse': -60,   # Velocidad negativa = movimiento hacia atrás
            'alejarse': 60,      # Velocidad positiva = movimiento hacia adelante
            'giro_centrado': 35, # Velocidad para centrar objeto
            'busqueda': 30       # Velocidad de giro en búsqueda
        }

        self.UMBRALES = {
            'min_area': 1000,
            'umbral_centrado': 40,    # Reducido para mayor precisión
            'ancho_frame': 640,
            'zona_segura': 150        # Margen desde los bordes del frame
        }
        # ========================================================

        self.setup_motores()
        self.setup_camara()

    def setup_motores(self):
        # (Mismo código anterior)
        for pin in self.MOTOR_PINS.values():
            GPIO.setup(pin, GPIO.OUT)
            GPIO.output(pin, GPIO.LOW)

        self.pwm_a = GPIO.PWM(self.MOTOR_PINS['ENA'], 1000)
        self.pwm_b = GPIO.PWM(self.MOTOR_PINS['ENB'], 1000)
        self.pwm_a.start(0)
        self.pwm_b.start(0)

    def setup_camara(self):
        # (Mismo código anterior)
        self.picam2 = Picamera2()
        self.picam2.configure(self.picam2.create_preview_configuration(
            main=self.CAMERA_CONFIG))
        self.picam2.start()

    def control_motores(self, izq, der):
        # (Mismo código anterior)
        izq = max(min(izq, 100), -100)
        der = max(min(der, 100), -100)

        GPIO.output(self.MOTOR_PINS['IN1'], GPIO.HIGH if izq >=0 else GPIO.LOW)
        GPIO.output(self.MOTOR_PINS['IN2'], GPIO.LOW if izq >=0 else GPIO.HIGH)
        self.pwm_a.ChangeDutyCycle(abs(izq))

        GPIO.output(self.MOTOR_PINS['IN3'], GPIO.HIGH if der >=0 else GPIO.LOW)
        GPIO.output(self.MOTOR_PINS['IN4'], GPIO.LOW if der >=0 else GPIO.HIGH)
        self.pwm_b.ChangeDutyCycle(abs(der))

    def detectar_objetos(self):
        # (Mismo código anterior)
        frame = self.picam2.capture_array()
        frame = cv2.cvtColor(frame, cv2.COLOR_RGBA2BGR)
        hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)
        
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
            # Búsqueda adaptativa
            self.control_motores(-self.VELOCIDADES['busqueda'], self.VELOCIDADES['busqueda'])
            return False
        
        diferencia = centroide[0] - centro_x
        pos_x = centroide[0]

        # Evitar bordes del frame
        if pos_x < self.UMBRALES['zona_segura'] or pos_x > (self.UMBRALES['ancho_frame'] - self.UMBRALES['zona_segura']):
            self.control_motores(0, 0)
            time.sleep(0.5)
            return False

        if abs(diferencia) < self.UMBRALES['umbral_centrado']:
            # Movimiento hacia atrás para acercarse (cámara trasera)
            self.control_motores(self.VELOCIDADES['acercarse'], self.VELOCIDADES['acercarse'])
            return True
        else:
            # Giro proporcional
            velocidad_giro = int(self.VELOCIDADES['giro_centrado'] * (abs(diferencia)/centro_x))
            if diferencia < 0:
                # Objeto a la izquierda -> girar derecha
                self.control_motores(velocidad_giro, -velocidad_giro)
            else:
                # Objeto a la derecha -> girar izquierda
                self.control_motores(-velocidad_giro, velocidad_giro)
            return False

    def ejecutar(self):
        try:
            print("Iniciando sistema con cámara trasera...")
            while True:
                objetos = self.detectar_objetos()
                
                if objetos['green']:
                    if self.seguir_color(objetos['green']):
                        print("VERDE: Acercándose")
                    else:
                        print("VERDE: Ajustando posición")
                
                elif objetos['red']:
                    print("ROJO: Alejándose")
                    # Movimiento hacia adelante para alejarse
                    self.control_motores(self.VELOCIDADES['alejarse'], self.VELOCIDADES['alejarse'])
                
                else:
                    print("BUSCANDO: Girando en espiral")
                    # Patrón de búsqueda modificado
                    self.control_motores(
                        -int(self.VELOCIDADES['busqueda'] * 0.7),
                        self.VELOCIDADES['busqueda']
                    )

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
