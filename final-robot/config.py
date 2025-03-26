# Configuración de pines para Raspberry Pi
LED_PINS = {
    "avanzar": 23,
    "retroceder": 17,
    "izquierda": 18,
    "derecha": 27,
    "parar": 22,
}

MOTOR_PINS = {
    "int1": 3,
    "int2": 24,
    "ena_A": 26,
    "int3": 25,
    "int4": 2,
    "ena_B": 13,
}

SERVO_PINS = {
    "servo1": 5,  # Ajustar según conexión
    "servo2": 6,
}

# Configuración de PixyCam
PIXY_I2C_ADDRESS = 0x54  # Dirección predeterminada

# Parámetros del seguimiento de línea
LINE_THRESHOLD = 150  # Valor umbral para detección de línea

# Parámetros de detección de objetos
OBJECT_DETECTION_MIN_AREA = 500  # Área mínima para considerar un objeto

# Otros parámetros
LOG_LEVEL = "INFO"

