# main.py
# Versión final con gestión centralizada

from controllers.motors import MotorController
from sensors.line_sensor import LineSensor
from pid import PIDController
from config import VELOCIDAD_BASE
from utils.gpio_manager import GPIOManager

class LineFollower:
    def __init__(self):
        """Inicialización limpia sin duplicar GPIO"""
        self.motores = MotorController()
        self.sensores = LineSensor()
        self.pid = PIDController()

    def seguir_linea(self):
        """Loop principal optimizado"""
        try:
            while True:
                valores = self.sensores.leer_sensores()
                posicion = self.sensores.calcular_posicion(valores)
                
                if posicion is not None:
                    correccion = self.pid.calcular_correccion(posicion)
                    self.motores.control_motores(
                        VELOCIDAD_BASE - correccion,
                        VELOCIDAD_BASE + correccion
                    )

        except KeyboardInterrupt:
            self.detener()

    def detener(self):
        """Limpieza centralizada"""
        self.motores.detener()
        GPIOManager.cleanup()