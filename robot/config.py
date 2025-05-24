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

# Cámara y visión
CAMERA = {
    'resolution': (640, 480),
    'format': 'XRGB8888',
    'framerate': 30
}

# Detección de objetos
OBJECT_DETECTION = {
    'color_ranges': {
        'red': {
            'lower1': [0, 100, 100],
            'upper1': [10, 255, 255],
            'lower2': [160, 100, 100],
            'upper2': [180, 255, 255],
            'confidence': 1.5
        },
        'green': {
            'lower': [35, 100, 100],
            'upper': [85, 255, 255]
        }
    },
    'min_area': 1500,
    'safety_margin': 200
}

# Control de movimiento
MOVEMENT = {
    'approach_speed': -60,
    'centering_speed': 40,
    'search_speed': 30,
    'centering_threshold': 50
}

# Modos de operación
OPERATION_MODES = {
    'LINE_FOLLOWER': 0,
    'OBJECT_FOLLOWER': 1
}
