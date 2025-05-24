# utils/gpio_manager.py
# Gestor centralizado de GPIO para evitar conflictos

import RPi.GPIO as GPIO
import logging
from functools import wraps

class GPIOManager:
    _initialized = False
    _pin_usage = {}  # Dict para trackear pines en uso

    @classmethod
    def init(cls, mode=GPIO.BCM):
        """Configuración segura de GPIO (solo una vez)"""
        if not cls._initialized:
            try:
                GPIO.setmode(mode)
                GPIO.setwarnings(False)
                cls._initialized = True
                logging.info("GPIO inicializado en modo BCM")
            except Exception as e:
                logging.error(f"Error inicializando GPIO: {str(e)}")
                raise

    @classmethod
    def reserve_pin(cls, pin):
        """Registra un pin como en uso"""
        if pin in cls._pin_usage:
            raise ValueError(f"Pin {pin} ya está en uso por: {cls._pin_usage[pin]}")
        cls._pin_usage[pin] = "Reservado"

    @classmethod
    def release_pin(cls, pin):
        """Libera un pin registrado"""
        cls._pin_usage.pop(pin, None)

    @classmethod
    def cleanup(cls):
        """Limpia todos los pines y estado"""
        GPIO.cleanup()
        cls._initialized = False
        cls._pin_usage.clear()
        logging.info("GPIO cleanup completado")

# Decorador para funciones que usan GPIO
def gpio_protected(func):
    @wraps(func)
    def wrapper(*args, **kwargs):
        GPIOManager.init()
        try:
            return func(*args, **kwargs)
        except Exception as e:
            logging.error(f"Error GPIO en {func.__name__}: {str(e)}")
            raise
    return wrapper
