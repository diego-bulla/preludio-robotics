import RPi.GPIO as GPIO
import time
import cv2
import numpy as np
from picamera2 import Picamera2

class RobotControl:
    def __init__(self):
        # ============= CONFIGURACIONES ACTUALIZADAS =============
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
            'size': (640, 480)
        }

        # Rangos de color HSV actualizados
        self.COLOR_RANGES = {
            'green': {
                'lower': [35, 100, 100],  # Más restrictivo
                'upper': [85, 255, 255]
            },
            'red': {
                'lower1': [0, 100, 100],   # Más restrictivo
                'upper1': [10, 255, 255],
                'lower2': [160, 100, 100], # Ajustado para mejor detección
                'upper2': [180, 255, 255]
            }
        }

        # Parámetros de control optimizados
        self.VELOCIDADES = {
            'acercarse': -60,    # Movimiento hacia atrás (cámara trasera)
            'giro_centrado': 40, # Velocidad para centrar objeto
            'busqueda': 30       # Velocidad de giro en búsqueda
        }

        self.UMBRALES = {
            'min_area': 1500,        # Área mínima aumentada para mayor precisión
            'umbral_centrado': 50,   # Margen para considerar el objeto centrado
            'ancho_frame': 640,
            'zona_segura': 200,      # Margen aumentado desde los bordes
            'confianza_rojo': 1.5   # Factor de confianza para detección de rojo
        }
        # ========================================================

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
        
        # Detectar colores con filtrado mejorado
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

        # Aplicar filtrado morfológico
        kernel = np.ones((5,5), np.uint8)
        green_mask = cv2.morphologyEx(green_mask, cv2.MORPH_OPEN, kernel)
        red_mask = cv2.morphologyEx(red_mask, cv2.MORPH_OPEN, kernel)

        # Encontrar centroides con verificación de área
        def obtener_centroide(mask, color):
            contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
            if contours:
                max_contour = max(contours, key=cv2.contourArea)
                area = cv2.contourArea(max_contour)
                if area > self.UMBRALES['min_area'] * (self.UMBRALES['confianza_rojo'] if color == 'red' else 1):
                    M = cv2.moments(max_contour)
                    if M["m00"] != 0:
                        return (int(M["m10"] / M["m00"]), int(M["m01"] / M["m00"]), area)
            return None

        return {
            'green': obtener_centroide(green_mask, 'green'),
            'red': obtener_centroide(red_mask, 'red')
        }

    def seguir_color(self, centroide):
        if centroide is None:
            # Búsqueda adaptativa
            self.control_motores(-self.VELOCIDADES['busqueda'], self.VELOCIDADES['busqueda'])
            return False
        
        centro_x = self.UMBRALES['ancho_frame'] // 2
        pos_x, pos_y, area = centroide

        # Evitar bordes del frame
        if pos_x < self.UMBRALES['zona_segura'] or pos_x > (self.UMBRALES['ancho_frame'] - self.UMBRALES['zona_segura']):
            self.control_motores(0, 0)
            time.sleep(0.5)
            return False

        diferencia = pos_x - centro_x
        if abs(diferencia) < self.UMBRALES['umbral_centrado']:
            # Movimiento hacia atrás para acercarse
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
            print("Iniciando sistema de seguimiento mejorado...")
            while True:
                objetos = self.detectar_objetos()
                
                if objetos['green']:
                    if self.seguir_color(objetos['green']):
                        print("VERDE: Acercándose")
                    else:
                        print("VERDE: Ajustando posición")
                
                elif objetos['red']:
                    if self.seguir_color(objetos['red']):
                        print("ROJO: Acercándose")
                    else:
                        print("ROJO: Ajustando posición")
                
                else:
                    print("BUSCANDO: Girando en espiral")
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
