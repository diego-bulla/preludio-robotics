# controllers/motors.py

import logging
import RPi.GPIO as GPIO  # Necesario para usar GPIO directamente

from utils.gpio_manager import GPIOManager, gpio_protected
from config import MOTOR_PINS, FRECUENCIA_PWM

class MotorController:
    def __init__(self):
        GPIOManager.init()
        self._setup_pins()
        self._init_pwm()
        self.logger = logging.getLogger(__name__)
        self.logger.info("Controlador de motores listo")

    def _setup_pins(self):
        for pin_name, pin in MOTOR_PINS.items():
            GPIOManager.reserve_pin(pin)
            GPIO.setup(pin, GPIO.OUT)
            GPIO.output(pin, GPIO.LOW)

    def _init_pwm(self):
        self.pwm_a = GPIO.PWM(MOTOR_PINS["ENA"], FRECUENCIA_PWM)
        self.pwm_b = GPIO.PWM(MOTOR_PINS["ENB"], FRECUENCIA_PWM)
        self.pwm_a.start(0)
        self.pwm_b.start(0)

    @gpio_protected
    def control_motores(self, velocidad_izq, velocidad_der):
        vel_izq = max(min(round(velocidad_izq), 100), -100)
        vel_der = max(min(round(velocidad_der), 100), -100)

        GPIO.output(MOTOR_PINS["IN1"], vel_izq >= 0)
        GPIO.output(MOTOR_PINS["IN2"], vel_izq < 0)
        self.pwm_a.ChangeDutyCycle(abs(vel_izq))

        GPIO.output(MOTOR_PINS["IN3"], vel_der >= 0)
        GPIO.output(MOTOR_PINS["IN4"], vel_der < 0)
        self.pwm_b.ChangeDutyCycle(abs(vel_der))

    def avanzar(self, velocidad=50):
        """Avanza compensando motores enfrentados"""
        self.control_motores(+velocidad, -velocidad)

    def retroceder(self, velocidad=50):
        """Retrocede compensando motores enfrentados"""
        self.control_motores(-velocidad, +velocidad)

    def girar_izquierda(self, velocidad=50):
        """Giro en sentido antihorario"""
        self.control_motores(-velocidad, -velocidad)

    def girar_derecha(self, velocidad=50):
        """Giro en sentido horario"""
        self.control_motores(+velocidad, +velocidad)

    def detener(self):
        """Detiene ambos motores y libera GPIO"""
        self.pwm_a.stop()
        self.pwm_b.stop()
        GPIOManager.cleanup()
