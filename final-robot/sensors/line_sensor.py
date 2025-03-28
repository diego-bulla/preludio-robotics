# sensors/line_sensor.py
# Versión compatible con GPIOManager

import time
import logging
from utils.gpio_manager import GPIOManager, gpio_protected
from config import SENSOR_PINS, TIEMPO_LECTURA, UMBRAL_SENSOR, POSICION_CENTRAL

class LineSensor:
    def __init__(self):
        """Inicialización con registro centralizado de pines"""
        GPIOManager.init()
        self.logger = logging.getLogger(__name__)
        self._register_pins()

    def _register_pins(self):
        """Registra pines en GPIOManager"""
        for pin in SENSOR_PINS:
            GPIOManager.reserve_pin(pin)
            GPIO.setup(pin, GPIO.IN)

    @gpio_protected
    def leer_sensores(self):
        """Lectura protegida con decorador"""
        return [self._leer_sensor(pin) for pin in SENSOR_PINS]

    def _leer_sensor(self, pin):
        """Método interno para lectura individual"""
        try:
            GPIO.setup(pin, GPIO.OUT)
            GPIO.output(pin, GPIO.HIGH)
            time.sleep(0.00001)
            
            GPIO.setup(pin, GPIO.IN)
            inicio = time.time()
            while GPIO.input(pin) and (time.time() - inicio) < TIEMPO_LECTURA:
                pass
                
            return (time.time() - inicio) < UMBRAL_SENSOR
        except Exception as e:
            self.logger.error(f"Error en sensor {pin}: {str(e)}")
            return False

    def calcular_posicion(self, valores_sensores):
        """Cálculo de posición con manejo de errores"""
        if not any(valores_sensores):
            return None
            
        try:
            active_sensors = [i for i, v in enumerate(valores_sensores) if v]
            return sum(active_sensors) * 1000 / len(active_sensors) if active_sensors else POSICION_CENTRAL
        except Exception as e:
            self.logger.error(f"Error cálculo posición: {str(e)}")
            return POSICION_CENTRAL