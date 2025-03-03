import RPi.GPIO as GPIO
import time
import cv2
import numpy as np
from picamera2 import Picamera2

class RobotControl:
    def __init__(self):
        # ============= CONFIGURACIÓN AVANZADA =============
        GPIO.setmode(GPIO.BCM)
        GPIO.setwarnings(False)

        # Pines de motores (asegurar compatibilidad con tu hardware)
        self.MOTOR_PINS = {
            'ENA': 26,  # PWM Motor Izquierdo
            'IN1': 19,  # Dirección Motor Izquierdo
            'IN2': 13,
            'ENB': 11,  # PWM Motor Derecho
            'IN3': 6,   # Dirección Motor Derecho
            'IN4': 5
        }

        # Configuración cámara
        self.CAMERA_CONFIG = {
            'format': 'XRGB8888',
            'size': (640, 480)
        }

        # Rangos HSV optimizados (ajustar según ambiente)
        self.COLOR_RANGES = {
            'green': {
                'lower': [40, 80, 60],
                'upper': [80, 255, 255],
                'kernel': np.ones((7,7), np.uint8)
            },
            'red': {
                'lower1': [0, 120, 70],
                'upper1': [8, 255, 255],
                'lower2': [172, 120, 70],
                'upper2': [180, 255, 255],
                'kernel': np.ones((9,9), np.uint8)
            }
        }

        # Parámetros de control PID (ajustar experimentalmente)
        self.CONTROL = {
            'base_speed': 70,       # Velocidad base PWM (0-100)
            'max_speed': 100,
            'min_speed': 40,
            'Kp': 0.45,            # Proporcional
            'Ki': 0.001,           # Integral
            'Kd': 0.08,             # Derivativo
            'dead_zone': 15,       # Zona muerta en pixeles
            'area_objetivo': 30000, # Área para distancia óptima
            'max_area': 50000
        }

        self.UMBRALES = {
            'min_area': 2000,
            'ancho_frame': 640,
            'confianza_rojo': 1.2
        }
        # =================================================

        self.setup_hardware()
        self.pid_reset()
        
    def setup_hardware(self):
        # Configuración de pines y PWM
        for pin in self.MOTOR_PINS.values():
            GPIO.setup(pin, GPIO.OUT)
            GPIO.output(pin, GPIO.LOW)

        self.pwm_left = GPIO.PWM(self.MOTOR_PINS['ENA'], 2000)  # Frecuencia más alta
        self.pwm_right = GPIO.PWM(self.MOTOR_PINS['ENB'], 2000)
        self.pwm_left.start(0)
        self.pwm_right.start(0)

        # Configurar cámara
        self.picam2 = Picamera2()
        self.picam2.configure(self.picam2.create_preview_configuration(main=self.CAMERA_CONFIG))
        self.picam2.start()

    def pid_reset(self):
        self.integral = 0
        self.last_error = 0
        self.last_time = time.time()

    def control_motores(self, left, right):
        # Aplicar límites y direcciones
        left = np.clip(left, -100, 100)
        right = np.clip(right, -100, 100)

        # Motor izquierdo
        GPIO.output(self.MOTOR_PINS['IN1'], GPIO.HIGH if left >=0 else GPIO.LOW)
        GPIO.output(self.MOTOR_PINS['IN2'], GPIO.LOW if left >=0 else GPIO.HIGH)
        self.pwm_left.ChangeDutyCycle(abs(int(left)))

        # Motor derecho
        GPIO.output(self.MOTOR_PINS['IN3'], GPIO.HIGH if right >=0 else GPIO.LOW)
        GPIO.output(self.MOTOR_PINS['IN4'], GPIO.LOW if right >=0 else GPIO.HIGH)
        self.pwm_right.ChangeDutyCycle(abs(int(right)))

    def procesar_color(self, mask, color_config):
        # Mejorar la detección con operaciones morfológicas
        mask = cv2.morphologyEx(mask, cv2.MORPH_OPEN, color_config['kernel'])
        mask = cv2.morphologyEx(mask, cv2.MORPH_CLOSE, color_config['kernel'])
        
        contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
        if not contours:
            return None

        # Filtrar por área y relación de aspecto
        max_contour = max(contours, key=lambda c: cv2.contourArea(c))
        area = cv2.contourArea(max_contour)
        x, y, w, h = cv2.boundingRect(max_contour)
        aspect_ratio = w / h

        if area > self.UMBRALES['min_area'] and 0.5 < aspect_ratio < 2:
            M = cv2.moments(max_contour)
            if M["m00"] != 0:
                return (int(M["m10"]/M["m00"]), int(M["m01"]/M["m00"]), area
        return None

    def detectar_objetos(self):
        frame = self.picam2.capture_array()
        frame = cv2.cvtColor(frame, cv2.COLOR_RGBA2BGR)
        hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)

        # Procesar verde
        green_mask = cv2.inRange(hsv,
            np.array(self.COLOR_RANGES['green']['lower']),
            np.array(self.COLOR_RANGES['green']['upper']))
        green_data = self.procesar_color(green_mask, self.COLOR_RANGES['green'])

        # Procesar rojo
        red_mask1 = cv2.inRange(hsv,
            np.array(self.COLOR_RANGES['red']['lower1']),
            np.array(self.COLOR_RANGES['red']['upper1']))
        red_mask2 = cv2.inRange(hsv,
            np.array(self.COLOR_RANGES['red']['lower2']),
            np.array(self.COLOR_RANGES['red']['upper2']))
        red_mask = cv2.bitwise_or(red_mask1, red_mask2)
        red_data = self.procesar_color(red_mask, self.COLOR_RANGES['red'])

        return {'green': green_data, 'red': red_data}

    def calcular_velocidades(self, error, area):
        # Control PID
        now = time.time()
        dt = now - self.last_time
        
        self.integral += error * dt
        derivative = (error - self.last_error) / dt if dt > 0 else 0
        
        # Cálculo de salida PID
        output = (self.CONTROL['Kp'] * error +
                 self.CONTROL['Ki'] * self.integral +
                 self.CONTROL['Kd'] * derivative)
        
        # Control de velocidad basado en área
        area_factor = np.clip(area / self.CONTROL['area_objetivo'], 0.5, 1.5)
        base_speed = self.CONTROL['base_speed'] * area_factor
        
        # Aplicar zona muerta
        if abs(error) < self.CONTROL['dead_zone']:
            return base_speed, base_speed
        
        # Velocidades diferenciales
        left_speed = base_speed + output
        right_speed = base_speed - output
        
        # Limitar velocidades
        left_speed = np.clip(left_speed, self.CONTROL['min_speed'], self.CONTROL['max_speed'])
        right_speed = np.clip(right_speed, self.CONTROL['min_speed'], self.CONTROL['max_speed'])
        
        self.last_error = error
        self.last_time = now
        
        return left_speed, right_speed

    def seguir_objeto(self, target):
        if target is None:
            self.control_motores(-self.CONTROL['base_speed'], self.CONTROL['base_speed'])
            return False
        
        centro_x = self.UMBRALES['ancho_frame'] // 2
        obj_x, obj_y, area = target
        
        error = centro_x - obj_x
        left_speed, right_speed = self.calcular_velocidades(error, area)
        
        # Invertir velocidades porque la cámara está trasera
        self.control_motores(-left_speed, -right_speed)
        
        return abs(error) < self.CONTROL['dead_zone']

    def ejecutar(self):
        try:
            print("Sistema iniciado - Modo seguimiento mejorado")
            while True:
                objetos = self.detectar_objetos()
                objetivo = None
                
                # Prioridad: verde -> rojo
                if objetos['green']:
                    objetivo = objetos['green']
                    print("Objetivo verde detectado")
                elif objetos['red'] and objetos['red'][2] > self.UMBRALES['min_area'] * self.UMBRALES['confianza_rojo']:
                    objetivo = objetos['red']
                    print("Objetivo rojo detectado")
                
                if objetivo:
                    if self.seguir_objeto(objetivo):
                        print("Objetivo centrado - Manteniendo posición")
                    else:
                        print("Ajustando posición...")
                else:
                    print("Buscando objetivo...")
                    self.control_motores(-40, 40)  # Giro suave de búsqueda
                
                time.sleep(0.05)

        except KeyboardInterrupt:
            self.detener()

    def detener(self):
        self.control_motores(0, 0)
        self.pwm_left.stop()
        self.pwm_right.stop()
        self.picam2.stop()
        GPIO.cleanup()
        print("\nSistema detenido correctamente")

if __name__ == "__main__":
    robot = RobotControl()
    robot.ejecutar()
