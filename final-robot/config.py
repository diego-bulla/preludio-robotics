# config.py
# Archivo de configuración con los pines y parámetros del robot

# Configuración de pines para el motor (L298N)
MOTOR_PINS = {
    "ENA": 13,  # PWM motor A
    "IN1": 19,  # Control dirección motor A
    "IN2": 16,
    "ENB": 21,  # PWM motor B
    "IN3": 26,   # Control dirección motor B
    "IN4": 20
}

# Configuración de pines para sensores de línea (QTR-8RC)
SENSOR_PINS = [2, 3, 4, 17, 18, 27]

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

