# controllers/motor.py
# Controlador de motores usando el driver L298N

import RPi.GPIO as GPIO
import time
from config import MOTOR_PINS  # Importar pines desde config.py

class MotorController:
    def __init__(self):
        GPIO.setmode(GPIO.BCM)
        GPIO.setwarnings(False)

        # Configurar pines de motores como salida
        for pin in MOTOR_PINS.values():
            GPIO.setup(pin, GPIO.OUT)

        # Crear objetos PWM
        self.pwm_a = GPIO.PWM(MOTOR_PINS["ENA"], 1000)  # 1000 Hz
        self.pwm_b = GPIO.PWM(MOTOR_PINS["ENB"], 1000)
        self.pwm_a.start(0)
        self.pwm_b.start(0)

    def mover(self, velocidad_izq, velocidad_der):
        """Controla la dirección y velocidad de los motores."""
        velocidad_izq = max(min(velocidad_izq, 100), -100)
        velocidad_der = max(min(velocidad_der, 100), -100)

        # Motor izquierdo
        GPIO.output(MOTOR_PINS["IN1"], velocidad_izq >= 0)
        GPIO.output(MOTOR_PINS["IN2"], velocidad_izq < 0)
        self.pwm_a.ChangeDutyCycle(abs(velocidad_izq))

        # Motor derecho
        GPIO.output(MOTOR_PINS["IN3"], velocidad_der >= 0)
        GPIO.output(MOTOR_PINS["IN4"], velocidad_der < 0)
        self.pwm_b.ChangeDutyCycle(abs(velocidad_der))

    def detener(self):
        """Detiene los motores y limpia el PWM."""
        self.mover(0, 0)
        self.pwm_a.stop()
        self.pwm_b.stop()
        GPIO.cleanup()

