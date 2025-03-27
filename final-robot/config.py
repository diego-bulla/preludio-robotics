# config.py
# Archivo de configuración con los pines y parámetros del robot

# Configuración de pines para el motor (L298N)
MOTOR_PINS = {
    "ENA": 26,  # PWM motor A
    "IN1": 19,  # Control dirección motor A
    "IN2": 13,
    "ENB": 11,  # PWM motor B
    "IN3": 6,   # Control dirección motor B
    "IN4": 5
}

# Configuración de pines para sensores de línea (QTR-8RC)
SENSOR_PINS = [24, 25, 8, 7, 12, 16, 20, 21]

# Configuración de control de movimiento
VELOCIDAD_BASE = 50  # Velocidad base del robot (0-100)
TIEMPO_LECTURA = 0.1  # Tiempo para lectura de sensores (segundos)
UMBRAL_SENSOR = 0.5  # Umbral para detectar línea negra

# Configuración de PID
PID_CONSTANTES = {
    "Kp": 0.5,
    "Ki": 0.0001,
    "Kd": 0.1,
    "INTEGRAL_LIMITE": 0
}

