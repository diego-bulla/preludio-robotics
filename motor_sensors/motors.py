# controllers/motors.py
# Versión compatible con GPIOManager

import logging
from utils.gpio_manager import GPIOManager, gpio_protected
from config import MOTOR_PINS, FRECUENCIA_PWM

class MotorController:
    def __init__(self):
        """Inicialización segura con gestión centralizada de GPIO"""
        GPIOManager.init()  # Inicializa el gestor de GPIO para manejar los pines de forma segura.
        self._setup_pins()  # Configura y registra los pines de los motores.
        self._init_pwm()  # Inicializa la modulación por ancho de pulso (PWM) para controlar la velocidad.
        self.logger = logging.getLogger(__name__)  # Crea un logger para registrar eventos.
        self.logger.info("Controlador de motores listo")  # Registra que el controlador está listo.

    def _setup_pins(self):
        """Registra y configura pines"""
        for pin_name, pin in MOTOR_PINS.items():
            GPIOManager.reserve_pin(pin)  # Reserva el pin para evitar conflictos.
            GPIO.setup(pin, GPIO.OUT)  # Configura el pin como salida.
            GPIO.output(pin, GPIO.LOW)  # Inicializa el pin en estado bajo.

    def _init_pwm(self):
        """Inicialización protegida de PWM"""
        self.pwm_a = GPIO.PWM(MOTOR_PINS["ENA"], FRECUENCIA_PWM)  # Crea un objeto PWM para el motor A.
        self.pwm_b = GPIO.PWM(MOTOR_PINS["ENB"], FRECUENCIA_PWM)  # Crea un objeto PWM para el motor B.
        self.pwm_a.start(0)  # Inicia el PWM del motor A con un ciclo de trabajo del 0%.
        self.pwm_b.start(0)  # Inicia el PWM del motor B con un ciclo de trabajo del 0%.

    @gpio_protected
    def control_motores(self, velocidad_izq, velocidad_der):
        """Control con protección GPIO"""
        vel_izq = max(min(round(velocidad_izq), 100), -100)  # Limita la velocidad izquierda entre -100 y 100.
        vel_der = max(min(round(velocidad_der), 100), -100)  # Limita la velocidad derecha entre -100 y 100.

        GPIO.output(MOTOR_PINS["IN1"], vel_izq >= 0)  # Activa la dirección del motor A según la velocidad.
        GPIO.output(MOTOR_PINS["IN2"], vel_izq < 0)  # Activa la dirección opuesta si la velocidad es negativa.
        self.pwm_a.ChangeDutyCycle(abs(vel_izq))  # Cambia el ciclo de trabajo del motor A según la velocidad.

        GPIO.output(MOTOR_PINS["IN3"], vel_der >= 0)  # Activa la dirección del motor B según la velocidad.
        GPIO.output(MOTOR_PINS["IN4"], vel_der < 0)  # Activa la dirección opuesta si la velocidad es negativa.
        self.pwm_b.ChangeDutyCycle(abs(vel_der))  # Cambia el ciclo de trabajo del motor B según la velocidad.

    def detener(self):
        """Limpieza segura"""
        self.pwm_a.stop()  # Detiene el PWM del motor A.
        self.pwm_b.stop()  # Detiene el PWM del motor B.
        GPIOManager.cleanup()  # Limpia y libera los recursos de GPIO.
