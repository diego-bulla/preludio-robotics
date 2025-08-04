#!/usr/bin/env python3
# config.py - Configuraciones del Robot Autónomo

import os

# === INFORMACIÓN DEL PROYECTO ===
PROJECT_NAME = "Robot Autónomo"
VERSION = "1.0.0"
AUTHOR = "Tu Nombre"

# === CONFIGURACIONES DE HARDWARE ===

# --- PINES GPIO RASPBERRY PI ---
# Motores L298N
MOTOR_LEFT_IN1 = 18    # GPIO 18 (Pin 12)
MOTOR_LEFT_IN2 = 19    # GPIO 19 (Pin 35) 
MOTOR_RIGHT_IN3 = 20   # GPIO 20 (Pin 38)
MOTOR_RIGHT_IN4 = 21   # GPIO 21 (Pin 40)

# PWM para control de velocidad
MOTOR_LEFT_PWM = 12    # GPIO 12 (Pin 32) - ENA
MOTOR_RIGHT_PWM = 13   # GPIO 13 (Pin 33) - ENB

# Encoders magnéticos
ENCODER_LEFT_A = 2     # GPIO 2 (Pin 3)
ENCODER_LEFT_B = 3     # GPIO 3 (Pin 5)
ENCODER_RIGHT_A = 4    # GPIO 4 (Pin 7)
ENCODER_RIGHT_B = 17   # GPIO 17 (Pin 11)

# --- CONFIGURACIONES DE COMUNICACIÓN SERIAL ---
ARDUINO_PORT = "/dev/ttyUSB0"    # Puerto serial Arduino (cambiar según sistema)
ARDUINO_BAUDRATE = 115200        # Velocidad de comunicación
SERIAL_TIMEOUT = 1.0             # Timeout de lectura (segundos)
SERIAL_WRITE_TIMEOUT = 1.0       # Timeout de escritura (segundos)
MAX_CONSECUTIVE_ERRORS = 10      # Máximo errores consecutivos antes de reconectar

# === CONFIGURACIONES DE MOTORES ===
PWM_FREQUENCY = 1000             # Frecuencia PWM en Hz
MAX_SPEED = 100                  # Velocidad máxima (0-100%)
MIN_SPEED = 20                   # Velocidad mínima para movimiento
DEFAULT_SPEED = 50               # Velocidad por defecto

# Dirección de motores - ajustar según conexiones físicas
# Si el robot se mueve al revés, cambiar estos valores
MOTOR_LEFT_FORWARD = 1           # 1 = normal, -1 = invertir dirección
MOTOR_RIGHT_FORWARD = -1         # 1 = normal, -1 = invertir dirección

# === CONFIGURACIONES DE ENCODERS ===
PULSES_PER_REVOLUTION = 12       # Pulsos por revolución del encoder magnético
ENCODER_QUADRATURE = True        # True si usas ambos canales A y B (cuadratura)
ENCODER_DEBOUNCE_TIME = 0.001    # Tiempo de debounce en segundos

# === CONFIGURACIONES FÍSICAS DEL ROBOT ===
# Dimensiones de ruedas
WHEEL_DIAMETER_CM = 6.5          # Diámetro de la rueda en cm
WHEEL_RADIUS_CM = WHEEL_DIAMETER_CM / 2.0
WHEEL_CIRCUMFERENCE_CM = WHEEL_DIAMETER_CM * 3.14159

# Distancia entre ruedas (wheelbase) en cm
WHEELBASE_CM = 15.0              # Distancia entre centros de las ruedas

# Dimensiones del robot
ROBOT_LENGTH_CM = 20.0           # Largo del robot
ROBOT_WIDTH_CM = 18.0            # Ancho del robot
ROBOT_HEIGHT_CM = 10.0           # Alto del robot

# === CONFIGURACIONES DE SENSORES ULTRASÓNICOS ===
ULTRASONIC_MAX_DISTANCE = 400    # Distancia máxima de detección (cm)
ULTRASONIC_MIN_DISTANCE = 2     # Distancia mínima de detección (cm)
ULTRASONIC_FREQUENCY = 10        # Frecuencia de lectura (Hz)

# Posiciones de sensores en el robot (cm desde el centro)
SENSOR_POSITIONS = {
    'front': {'x': ROBOT_LENGTH_CM/2, 'y': 0, 'angle': 0},
    'back':  {'x': -ROBOT_LENGTH_CM/2, 'y': 0, 'angle': 180},
    'left':  {'x': 0, 'y': ROBOT_WIDTH_CM/2, 'angle': 90},
    'right': {'x': 0, 'y': -ROBOT_WIDTH_CM/2, 'angle': -90}
}

# === CONFIGURACIONES DE CÁMARA ===
CAMERA_DEVICE = 0                # Dispositivo de cámara (0 = primera USB)
CAMERA_WIDTH = 640               # Resolución horizontal
CAMERA_HEIGHT = 480              # Resolución vertical
CAMERA_FPS = 30                  # Frames por segundo
CAMERA_ENABLED = True            # Habilitar/deshabilitar cámara

# === CONFIGURACIONES DE NAVEGACIÓN ===
# Distancias de seguridad
OBSTACLE_THRESHOLD_CM = 30       # Distancia mínima para considerar obstáculo
DANGER_THRESHOLD_CM = 15         # Distancia de peligro (parada de emergencia)
SAFE_DISTANCE_CM = 50            # Distancia segura para navegación

# Velocidades para diferentes situaciones
SPEED_NORMAL = 60                # Velocidad normal de navegación
SPEED_SLOW = 30                  # Velocidad lenta (cerca de obstáculos)
SPEED_TURN = 40                  # Velocidad para giros
SPEED_REVERSE = 35               # Velocidad para retroceso

# === CONFIGURACIONES DE IA/SENSOR FUSION ===
# Frecuencias de procesamiento
AI_UPDATE_FREQUENCY = 10         # Frecuencia de decisiones de IA (Hz)
SENSOR_FUSION_FREQUENCY = 20     # Frecuencia de fusión de sensores (Hz)
ODOMETRY_UPDATE_FREQUENCY = 50   # Frecuencia de actualización de odometría (Hz)

# Pesos para sensor fusion
ULTRASONIC_WEIGHT = 0.7          # Peso de sensores ultrasónicos
ODOMETRY_WEIGHT = 0.3            # Peso de odometría
CAMERA_WEIGHT = 0.5              # Peso de información de cámara (si disponible)

# Filtro de Kalman - parámetros básicos
KALMAN_PROCESS_NOISE = 0.1       # Ruido del proceso
KALMAN_MEASUREMENT_NOISE = 1.0   # Ruido de medición
KALMAN_INITIAL_UNCERTAINTY = 10.0 # Incertidumbre inicial

# === CONFIGURACIONES DE PRUEBAS ===
TEST_SPEED = 50                  # Velocidad para pruebas (0-100%)
TEST_DURATION = 2.0              # Duración de cada movimiento en segundos
PRINT_INTERVAL = 0.1             # Intervalo para imprimir datos en segundos

# Configuraciones de pruebas específicas
TEST_DISTANCE_CM = 20            # Distancia objetivo para pruebas de precisión
TEST_ROTATION_DEGREES = 90       # Ángulo objetivo para pruebas de rotación

# === CONFIGURACIONES DE LOGGING ===
LOG_LEVEL = "INFO"               # DEBUG, INFO, WARNING, ERROR
LOG_TO_FILE = True               # Guardar logs en archivo
LOG_TO_CONSOLE = True            # Mostrar logs en consola
LOG_FILE_PATH = "logs/robot.log" # Ruta del archivo de log
MAX_LOG_SIZE_MB = 10             # Tamaño máximo del archivo de log
LOG_BACKUP_COUNT = 5             # Número de archivos de backup

# === CONFIGURACIONES DEL SISTEMA ===
# Threads y procesos
MAX_THREADS = 8                  # Máximo número de threads
THREAD_TIMEOUT = 5.0             # Timeout para operaciones de thread

# Recursos del sistema
MAX_CPU_USAGE = 80               # Máximo uso de CPU permitido (%)
MAX_MEMORY_USAGE = 80            # Máximo uso de memoria permitido (%)

# === CONFIGURACIONES DE SEGURIDAD ===
EMERGENCY_STOP_ENABLED = True    # Habilitar parada de emergencia
WATCHDOG_TIMEOUT = 5.0           # Timeout del watchdog (segundos)
MAX_OPERATION_TIME = 300         # Tiempo máximo de operación continua (segundos)

# Límites de seguridad
MAX_ACCELERATION = 100           # Máxima aceleración permitida (cm/s²)
MAX_ANGULAR_VELOCITY = 180       # Máxima velocidad angular (grados/s)

# === CONFIGURACIONES DE ARCHIVO ===
# Directorios
BASE_DIR = os.path.dirname(os.path.abspath(__file__))
LOGS_DIR = os.path.join(BASE_DIR, "logs")
DATA_DIR = os.path.join(BASE_DIR, "data")
MODELS_DIR = os.path.join(BASE_DIR, "models")

# Crear directorios si no existen
for directory in [LOGS_DIR, DATA_DIR, MODELS_DIR]:
    os.makedirs(directory, exist_ok=True)

# === CONFIGURACIONES ESPECÍFICAS DE LA COMPETENCIA ===
# Basado en el rulebook de "Robotics for Good Youth Challenge"
COMPETITION_MODE = False         # Modo competencia activado

# Configuraciones específicas de la competencia
GAME_DURATION = 120              # Duración del juego en segundos
FIELD_WIDTH_CM = 1171            # Ancho del campo de competencia
FIELD_HEIGHT_CM = 1143           # Alto del campo de competencia

# Elementos del juego
RED_BLOCK_SIZE_CM = 3.0          # Tamaño de bloques rojos (heridos)
GREEN_BLOCK_SIZE_CM = 3.0        # Tamaño de bloques verdes (evacuados)
BROWN_BLOCK_SIZE_CM = 12.0       # Tamaño de bloques marrones (escombros)

# Zonas importantes
HOSPITAL_ZONE = "lower_left"     # Ubicación de la zona hospital
REFUGE_ZONE = "lower_right"      # Ubicación de la zona refugio
START_ZONE_DEPTH_CM = 30         # Profundidad de la zona de inicio

# === FUNCIONES DE UTILIDAD ===
def get_config_summary():
    """Obtener resumen de configuración para debugging"""
    summary = {
        'project': f"{PROJECT_NAME} v{VERSION}",
        'motors': f"PWM: {PWM_FREQUENCY}Hz, Max Speed: {MAX_SPEED}%",
        'encoders': f"PPR: {PULSES_PER_REVOLUTION}, Wheel: {WHEEL_DIAMETER_CM}cm",
        'communication': f"Port: {ARDUINO_PORT}, Baud: {ARDUINO_BAUDRATE}",
        'sensors': f"Range: {ULTRASONIC_MIN_DISTANCE}-{ULTRASONIC_MAX_DISTANCE}cm, Freq: {ULTRASONIC_FREQUENCY}Hz",
        'robot_size': f"{ROBOT_LENGTH_CM}x{ROBOT_WIDTH_CM}x{ROBOT_HEIGHT_CM}cm",
        'wheelbase': f"{WHEELBASE_CM}cm"
    }
    return summary

def validate_config():
    """Validar configuración básica"""
    errors = []
    
    # Validar rangos de velocidad
    if not (0 <= MIN_SPEED <= MAX_SPEED <= 100):
        errors.append("Velocidades inválidas: MIN_SPEED <= MAX_SPEED <= 100")
    
    # Validar dimensiones físicas
    if WHEEL_DIAMETER_CM <= 0 or WHEELBASE_CM <= 0:
        errors.append("Dimensiones físicas deben ser positivas")
    
    # Validar frecuencias
    if PWM_FREQUENCY <= 0 or ULTRASONIC_FREQUENCY <= 0:
        errors.append("Frecuencias deben ser positivas")
    
    # Validar puerto serial
    if not ARDUINO_PORT:
        errors.append("Puerto serial de Arduino no configurado")
    
    return errors

def print_config():
    """Imprimir configuración actual"""
    print("🤖 === CONFIGURACIÓN DEL ROBOT ===")
    summary = get_config_summary()
    for key, value in summary.items():
        print(f"   {key.capitalize()}: {value}")
    
    # Validar configuración
    errors = validate_config()
    if errors:
        print("\n❌ ERRORES DE CONFIGURACIÓN:")
        for error in errors:
            print(f"   - {error}")
    else:
        print("\n✅ Configuración válida")

if __name__ == "__main__":
    print_config()