# config.py
# Configuración centralizada del robot

# --- Pines GPIO ---
MOTOR_PINS = {
    "ENA": 13,  # PWM motor A
    "IN1": 19,  # Control dirección motor A
    "IN2": 16,
    "ENB": 21,  # PWM motor B
    "IN3": 26,  # Control dirección motor B
    "IN4": 20
}

SENSOR_PINS = [2, 3, 4, 17, 18, 27]  # Pines para QTR-8RC

# --- Parámetros de control ---
VELOCIDAD_BASE = 50           # Velocidad base (0-100%)
TIEMPO_LECTURA = 0.1          # Tiempo máximo de lectura de sensores (segundos)
UMBRAL_SENSOR = 0.5           # Umbral para detectar línea (True/False)
POSICION_CENTRAL = 3500       # Valor de referencia para el centro de la línea
FRECUENCIA_PWM = 500          # Frecuencia PWM para motores (Hz)

# --- Constantes PID ---
PID_CONSTANTES = {
    "Kp": 0.5,                # Ganancia proporcional
    "Ki": 0.0001,             # Ganancia integral
    "Kd": 0.1,                # Ganancia derivativa
    "INTEGRAL_LIMITE": 100    # Límite anti-windup
}