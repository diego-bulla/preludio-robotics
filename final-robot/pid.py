# controllers/pid.py
# Módulo para control PID

from config import PID_CONSTANTES as PC

class PIDController:
    def __init__(self):
        self.error_anterior = 0
        self.integral = 0

    def calcular_correccion(self, posicion):
        """Calcula la corrección PID según la posición de la línea"""
        if posicion is None:
            return 0

        error = posicion - 3500  # Referencia al centro

        # Componentes PID
        p = PC["KP"] * error
        self.integral += error
        self.integral = max(min(self.integral, PC["INTEGRAL_LIMITE"]), -PC["INTEGRAL_LIMITE"])  # Anti-windup
        i = PC["KI"] * self.integral
        d = PC["KD"] * (error - self.error_anterior)

        self.error_anterior = error
        return p + i + d

