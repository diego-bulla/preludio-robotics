import RPi.GPIO as GPIO
import time
import cv2
import numpy as np
from picamera2 import Picamera2

class RobotControl:
    def __init__(self):
        # ============= CONFIGURACIÓN GPIO =============
        GPIO.setmode(GPIO.BCM)
        GPIO.setwarnings(False)
        
        # Pines de motores
        self.ENA = 26
        self.IN1 = 19
        self.IN2 = 13
        self.ENB = 11
        self.IN3 = 6
        self.IN4 = 5
        
        self.setup_motores()

        # ============= CONFIGURACIÓN CÁMARA =============
        self.picam2 = Picamera2()
        self.picam2.configure(self.picam2.create_preview_configuration(
            main={"format": 'XRGB8888', "size": (640, 480)}))
        self.picam2.start()

        # ============= PARÁMETROS AJUSTABLES =============
        # Rangos HSV mejorados
        self.COLOR_RANGES = {
            'green': {
                'lower': [40, 80, 80],
                'upper': [80, 255, 255]
            },
            'red': {
                'lower1': [0, 100, 100],
                'upper1': [8, 255, 255],
                'lower2': [172, 100, 100],
                'upper2': [179, 255, 255]
            }
        }
        
        # Control de motores (ajustado para cámara trasera)
        self.BASE_SPEED = -70        # Velocidad negativa para movimiento hacia atrás
        self.MIN_SPEED = 40          # Velocidad mínima absoluta
        self.ROTATION_SPEED = 60     # Velocidad de giro en búsqueda
        
        # Control PID para centrado
        self.Kp = 0.3
        self.Ki = 0.02
        self.Kd = 0.1
        self.last_error = 0
        self.integral = 0
        
        # Umbrales
        self.MIN_AREA = 1500         # Área mínima para detección
        self.CENTER_THRESHOLD = 20   # Margen de centrado (píxeles)
        
        # Tiempos
        self.SEARCH_DELAY = 0.1      # Tiempo entre ajustes en búsqueda

    def setup_motores(self):
        # Configuración de pines
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
        # Limitar velocidades y aplicar dirección invertida
        izq = max(min(izq, 100), -100)
        der = max(min(der, 100), -100)

        # Aplicar velocidad mínima cuando hay movimiento
        if izq != 0: izq = np.sign(izq) * max(abs(izq), self.MIN_SPEED)
        if der != 0: der = np.sign(der) * max(abs(der), self.MIN_SPEED)

        # Control motor izquierdo (invertido)
        GPIO.output(self.IN1, GPIO.LOW if izq >=0 else GPIO.HIGH)
        GPIO.output(self.IN2, GPIO.HIGH if izq >=0 else GPIO.LOW)
        self.pwm_a.ChangeDutyCycle(abs(izq))

        # Control motor derecho (invertido)
        GPIO.output(self.IN3, GPIO.LOW if der >=0 else GPIO.HIGH)
        GPIO.output(self.IN4, GPIO.HIGH if der >=0 else GPIO.LOW)
        self.pwm_b.ChangeDutyCycle(abs(der))

    def mejorar_deteccion(self, mask):
        # Mejorar la detección con filtrado morfológico
        kernel = np.ones((5,5), np.uint8)
        mask = cv2.morphologyEx(mask, cv2.MORPH_OPEN, kernel)
        mask = cv2.morphologyEx(mask, cv2.MORPH_CLOSE, kernel)
        return mask

    def detectar_objetos(self):
        frame = self.picam2.capture_array()
        frame = cv2.cvtColor(frame, cv2.COLOR_RGBA2BGR)
        hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)
        hsv = cv2.GaussianBlur(hsv, (5,5), 0)
        
        # Detección verde
        mask_green = cv2.inRange(hsv,
            np.array(self.COLOR_RANGES['green']['lower']),
            np.array(self.COLOR_RANGES['green']['upper']))
        mask_green = self.mejorar_deteccion(mask_green)
        
        # Detección roja
        mask_red1 = cv2.inRange(hsv,
            np.array(self.COLOR_RANGES['red']['lower1']),
            np.array(self.COLOR_RANGES['red']['upper1']))
        mask_red2 = cv2.inRange(hsv,
            np.array(self.COLOR_RANGES['red']['lower2']),
            np.array(self.COLOR_RANGES['red']['upper2']))
        mask_red = cv2.bitwise_or(mask_red1, mask_red2)
        mask_red = self.mejorar_deteccion(mask_red)

        # Encontrar mejor contorno
        def obtener_centroide(mask):
            contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
            if contours:
                max_contour = max(contours, key=cv2.contourArea)
                if cv2.contourArea(max_contour) > self.MIN_AREA:
                    M = cv2.moments(max_contour)
                    if M["m00"] != 0:
                        return (int(M["m10"] / M["m00"]), int(M["m01"] / M["m00"]))
            return None

        return {
            'green': obtener_centroide(mask_green),
            'red': obtener_centroide(mask_red)
        }

    def centrar_objeto(self, centro_x):
        ancho_frame = 640
        centro_objetivo = ancho_frame // 2
        error = centro_objetivo - centro_x
        
        # Control PID
        self.integral += error
        derivative = error - self.last_error
        correction = self.Kp * error + self.Ki * self.integral + self.Kd * derivative
        self.last_error = error
        
        # Aplicar corrección invertida (cámara trasera)
        left_speed = int(-self.BASE_SPEED + correction)
        right_speed = int(-self.BASE_SPEED - correction)
        
        # Limitar velocidades
        left_speed = np.clip(left_speed, -100, 100)
        right_speed = np.clip(right_speed, -100, 100)
        
        self.control_motores(left_speed, right_speed)
        return abs(error) < self.CENTER_THRESHOLD

    def buscar_objeto(self):
        # Giro suave en espiral (invertido)
        self.control_motores(self.ROTATION_SPEED, -self.ROTATION_SPEED)
        time.sleep(self.SEARCH_DELAY)

    def acercarse(self):
        # Movimiento hacia atrás (cámara trasera)
        self.control_motores(self.BASE_SPEED, self.BASE_SPEED)

    def ejecutar(self):
        try:
            print("Sistema iniciado - Buscando objetivos...")
            while True:
                objetos = self.detectar_objetos()
                target = None
                
                # Prioridad: verde primero
                if objetos['green']:
                    target = objetos['green']
                    print("Objeto verde detectado")
                elif objetos['red']:
                    target = objetos['red']
                    print("Objeto rojo detectado")
                
                if target:
                    if self.centrar_objeto(target[0]):
                        print("Objeto centrado - Avanzando")
                        self.acercarse()
                    else:
                        print("Ajustando posición...")
                else:
                    print("Buscando objetivo...")
                    self.buscar_objeto()

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
