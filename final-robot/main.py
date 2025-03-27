# routines/line_follower.py
# Rutina principal del seguidor de línea

import time
from controllers.motors import MotorController
from sensors.line_sensor import LineSensor
from pid import PIDController
from config import VELOCIDAD_BASE

class LineFollower:
    def __init__(self):
        self.motores = MotorController()
        self.sensores = LineSensor()
        self.pid = PIDController()

    def seguir_linea(self):
        """Ejecuta la rutina de seguimiento de línea"""
        print("Iniciando seguimiento de línea... Presiona Ctrl+C para detener.")

        try:
            while True:
                valores = self.sensores.leer_sensores()
                posicion = self.sensores.calcular_posicion(valores)

                if posicion is not None:
                    correccion = self.pid.calcular_correccion(posicion)

                    velocidad_izq = VELOCIDAD_BASE - correccion
                    velocidad_der = VELOCIDAD_BASE + correccion

                    self.motores.control_motores(velocidad_izq, velocidad_der)
                else:
                    self.motores.control_motores(0, 0)  # Detener si se pierde la línea

                time.sleep(0.01)

        except KeyboardInterrupt:
            self.detener()

    def detener(self):
        """Detiene los motores y limpia recursos"""
        self.motores.detener()
        print("\nRutina de seguimiento detenida.")

if __name__ == "__main__":
    robot = LineFollower()
    robot.seguir_linea()

