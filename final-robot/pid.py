# pid.py
# Versión con constantes correctas

from config import PID_CONSTANTES as PC

class PIDController:
    def __init__(self):
        self.error_anterior = 0
        self.integral = 0

    def calcular_correccion(self, posicion):
        """Cálculo PID protegido contra errores"""
        if posicion is None:
            return 0

        try:
            error = posicion - 3500  # POSICION_CENTRAL ya usada en cálculo previo
            self.integral = max(min(self.integral + error, PC["INTEGRAL_LIMITE"]), -PC["INTEGRAL_LIMITE"])
            
            return (
                PC["Kp"] * error +
                PC["Ki"] * self.integral +
                PC["Kd"] * (error - self.error_anterior)
            )
        except Exception as e:
            print(f"Error PID: {str(e)}")
            return 0