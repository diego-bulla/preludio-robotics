#!/usr/bin/env python3
"""
Sensor Fusion Simple - Control Directo Raspberry Pi
Combina: 4 HC-SR04 + MPU6050 + odometría estimada por PWM
Sin Arduino - Todo controlado por RPi
"""

import RPi.GPIO as GPIO
import time
import smbus
import threading
from math import sin, cos, radians, degrees, sqrt, atan2
import numpy as np

# ============================================================================
# CONFIGURACIÓN DE PINES
# ============================================================================

# L298N Motor Driver  
ENA = 13  # GPIO 13, Pin 33
IN1 = 19  # GPIO 19, Pin 35  
IN2 = 16  # GPIO 16, Pin 36
IN3 = 26  # GPIO 26, Pin 37
IN4 = 20  # GPIO 20, Pin 38
ENB = 21  # GPIO 21, Pin 40

# Sensores HC-SR04 (4 sensores)
# Sensor 1 (Frontal)
TRIG_SENSOR1 = 4   # GPIO 4, Pin 7
ECHO_SENSOR1 = 17  # GPIO 17, Pin 11

# Sensor 2 (Lateral Derecho)
TRIG_SENSOR2 = 27  # GPIO 27, Pin 13
ECHO_SENSOR2 = 22  # GPIO 22, Pin 15

# Sensor 3 (Lateral Izquierdo)
TRIG_SENSOR3 = 5   # GPIO 5, Pin 29
ECHO_SENSOR3 = 6   # GPIO 6, Pin 31

# Sensor 4 (Trasero)
TRIG_SENSOR4 = 12  # GPIO 12, Pin 32
ECHO_SENSOR4 = 24  # GPIO 24, Pin 18

# MPU6050
MPU6050_ADDR = 0x68
I2C_BUS = 1

# ============================================================================
# CONFIGURACIÓN DEL ROBOT
# ============================================================================

# Parámetros físicos
WHEEL_DIAMETER = 4.0    # cm
WHEEL_BASE = 15.0       # cm (distancia entre ruedas)

# Ángulos de sensores (respecto al frente del robot)
SENSOR_ANGLES = {
    'frontal': 0,           # Recto hacia adelante
    'lateral_der': 90,      # 90° hacia la derecha
    'lateral_izq': 270,     # 90° hacia la izquierda (270° = -90°)
    'trasero': 180          # 180° hacia atrás
}

# Parámetros del filtro
GYRO_ALPHA = 0.98           # Peso del giroscopio vs odometría
OBSTACLE_THRESHOLD = 20     # cm, distancia mínima a obstáculos
SPEED_CALIBRATION = 0.3     # Factor PWM->velocidad (ajustar experimentalmente)

GPIO.setmode(GPIO.BCM)
GPIO.setwarnings(False)

# ============================================================================
# CLASE PRINCIPAL: ROBOT CON SENSOR FUSION
# ============================================================================

class RobotController:
    def __init__(self):
        # Estado del robot
        self.x = 0.0            # Posición X (cm)
        self.y = 0.0            # Posición Y (cm) 
        self.heading = 0.0      # Orientación (grados)
        self.last_time = time.time()
        
        # Velocidades actuales (PWM)
        self.left_speed = 0
        self.right_speed = 0
        
        # Datos de sensores (4 sensores)
        self.distances = {
            'frontal': 999,
            'lateral_der': 999,
            'lateral_izq': 999,
            'trasero': 999
        }
        
        # Control de threading
        self.running = False
        self.data_lock = threading.Lock()
        
        # Inicializar hardware
        self._init_motors()
        self._init_sensors()
        self._init_mpu6050()
        
        print("🤖 Robot Controller iniciado - Control directo RPi con 4 sensores")

    def _init_motors(self):
        """Inicializar control de motores L298N"""
        try:
            # Configurar pines
            GPIO.setup([ENA, IN1, IN2, IN3, IN4, ENB], GPIO.OUT)
            
            # Crear PWM
            self.pwm_left = GPIO.PWM(ENA, 1000)   # 1kHz
            self.pwm_right = GPIO.PWM(ENB, 1000)
            
            self.pwm_left.start(0)
            self.pwm_right.start(0)
            
            print("✅ Motores L298N inicializados")
        except Exception as e:
            print(f"❌ Error iniciando motores: {e}")

    def _init_sensors(self):
        """Inicializar sensores HC-SR04"""
        try:
            # Configurar pines TRIG como OUTPUT
            trig_pins = [TRIG_SENSOR1, TRIG_SENSOR2, TRIG_SENSOR3, TRIG_SENSOR4]
            echo_pins = [ECHO_SENSOR1, ECHO_SENSOR2, ECHO_SENSOR3, ECHO_SENSOR4]
            
            for pin in trig_pins:
                GPIO.setup(pin, GPIO.OUT)
                GPIO.output(pin, GPIO.LOW)
                
            for pin in echo_pins:
                GPIO.setup(pin, GPIO.IN)
            
            print("✅ 4 Sensores HC-SR04 inicializados")
        except Exception as e:
            print(f"❌ Error iniciando sensores: {e}")

    def _init_mpu6050(self):
        """Inicializar MPU6050"""
        try:
            self.bus = smbus.SMBus(I2C_BUS)
            self.bus.write_byte_data(MPU6050_ADDR, 0x6B, 0)  # Despertar
            time.sleep(0.1)
            self.mpu_connected = True
            print("✅ MPU6050 inicializado")
        except Exception as e:
            self.mpu_connected = False
            print(f"❌ MPU6050 no disponible: {e}")

    # ========================================================================
    # LECTURA DE SENSORES
    # ========================================================================

    def read_ultrasonic(self, trig_pin, echo_pin):
        """Lee un sensor HC-SR04 individual"""
        try:
            # Enviar pulso
            GPIO.output(trig_pin, GPIO.HIGH)
            time.sleep(0.00001)  # 10µs
            GPIO.output(trig_pin, GPIO.LOW)
            
            # Medir tiempo con timeout
            timeout_start = time.time()
            
            # Esperar inicio del pulso
            while GPIO.input(echo_pin) == 0:
                pulse_start = time.time()
                if pulse_start - timeout_start > 0.1:
                    return -1
            
            # Esperar fin del pulso  
            while GPIO.input(echo_pin) == 1:
                pulse_end = time.time()
                if pulse_end - timeout_start > 0.1:
                    return -1
            
            # Calcular distancia
            pulse_duration = pulse_end - pulse_start
            distance = pulse_duration * 17150
            
            return distance if 2 <= distance <= 400 else -1
            
        except:
            return -1

    def read_all_sensors(self):
        """Lee todos los sensores HC-SR04 secuencialmente"""
        # Mapeo de sensores con sus pines
        sensor_configs = {
            'frontal': (TRIG_SENSOR1, ECHO_SENSOR1),
            'lateral_der': (TRIG_SENSOR2, ECHO_SENSOR2),
            'lateral_izq': (TRIG_SENSOR3, ECHO_SENSOR3),
            'trasero': (TRIG_SENSOR4, ECHO_SENSOR4)
        }
        
        # Leer sensores con pausa para evitar interferencia
        sensors_data = {}
        for sensor_name, (trig_pin, echo_pin) in sensor_configs.items():
            distance = self.read_ultrasonic(trig_pin, echo_pin)
            sensors_data[sensor_name] = distance
            time.sleep(0.02)  # 20ms entre sensores para evitar interferencia
        
        # Actualizar datos thread-safe
        with self.data_lock:
            for sensor, distance in sensors_data.items():
                if distance > 0:  # Solo actualizar si la lectura es válida
                    self.distances[sensor] = distance
        
        # Pausa entre ciclos completos
        time.sleep(0.05)

    def read_gyro_z(self):
        """Lee giroscopio Z para rotación"""
        if not self.mpu_connected:
            return 0
        
        try:
            high = self.bus.read_byte_data(MPU6050_ADDR, 0x47)
            low = self.bus.read_byte_data(MPU6050_ADDR, 0x48)
            
            value = (high << 8) | low
            if value > 32768:
                value = value - 65536
                
            return value / 131.0  # grados/segundo
        except:
            return 0

    # ========================================================================
    # CONTROL DE MOTORES
    # ========================================================================

    def move_motors(self, left_speed, right_speed):
        """Control de motores con límites"""
        # Limitar velocidades
        left_speed = max(-255, min(255, left_speed))
        right_speed = max(-255, min(255, right_speed))
        
        # Guardar velocidades actuales
        self.left_speed = left_speed
        self.right_speed = right_speed
        
        # Motor izquierdo (A)
        if left_speed >= 0:
            GPIO.output(IN1, GPIO.HIGH)
            GPIO.output(IN2, GPIO.LOW)
        else:
            GPIO.output(IN1, GPIO.LOW)
            GPIO.output(IN2, GPIO.HIGH)
            left_speed = -left_speed
            
        # Motor derecho (B)
        if right_speed >= 0:
            GPIO.output(IN3, GPIO.HIGH)
            GPIO.output(IN4, GPIO.LOW)
        else:
            GPIO.output(IN3, GPIO.LOW)
            GPIO.output(IN4, GPIO.HIGH)
            right_speed = -right_speed
        
        # Aplicar PWM (convertir 0-255 a 0-100%)
        self.pwm_left.ChangeDutyCycle(left_speed * 100 / 255)
        self.pwm_right.ChangeDutyCycle(right_speed * 100 / 255)

    def stop(self):
        """Detener robot"""
        self.move_motors(0, 0)

    # ========================================================================
    # ESTIMACIÓN DE POSICIÓN Y SENSOR FUSION
    # ========================================================================

    def update_odometry(self, dt):
        """Actualiza posición basada en velocidades PWM de motores"""
        # Convertir PWM a velocidad lineal estimada
        left_vel = (self.left_speed / 255.0) * SPEED_CALIBRATION * 100  # cm/s
        right_vel = (self.right_speed / 255.0) * SPEED_CALIBRATION * 100  # cm/s
        
        # Cinemática diferencial
        linear_vel = (left_vel + right_vel) / 2
        angular_vel_rad = (right_vel - left_vel) / WHEEL_BASE
        
        # Integrar posición
        self.x += linear_vel * cos(radians(self.heading)) * dt
        self.y += linear_vel * sin(radians(self.heading)) * dt
        
        return linear_vel, degrees(angular_vel_rad)

    def update_heading(self, dt):
        """Fusión de heading: giroscopio + odometría"""
        # Leer giroscopio
        gyro_rate = self.read_gyro_z()
        
        # Estimar rotación por odometría
        _, odo_angular_vel = self.update_odometry(dt)
        
        # Fusión complementaria
        gyro_change = gyro_rate * dt
        odo_change = odo_angular_vel * dt
        
        fused_change = GYRO_ALPHA * gyro_change + (1 - GYRO_ALPHA) * odo_change
        
        self.heading += fused_change
        
        # Normalizar ángulo [-180, 180]
        while self.heading > 180:
            self.heading -= 360
        while self.heading < -180:
            self.heading += 360

    def get_obstacles_global(self):
        """Convierte distancias locales a posiciones globales de obstáculos"""
        obstacles = []
        
        with self.data_lock:
            for sensor_name, distance in self.distances.items():
                if 2 <= distance <= 300:  # Rango válido
                    # Ángulo del sensor en coordenadas globales
                    sensor_angle = self.heading + SENSOR_ANGLES[sensor_name]
                    
                    # Posición del obstáculo
                    obs_x = self.x + distance * cos(radians(sensor_angle))
                    obs_y = self.y + distance * sin(radians(sensor_angle))
                    
                    obstacles.append({
                        'x': obs_x,
                        'y': obs_y,
                        'distance': distance,
                        'sensor': sensor_name,
                        'angle': sensor_angle
                    })
        
        return obstacles

    # ========================================================================
    # NAVEGACIÓN INTELIGENTE
    # ========================================================================

    def advanced_navigation(self):
        """Navegación reactiva mejorada con 4 sensores"""
        with self.data_lock:
            front_dist = self.distances['frontal']
            back_dist = self.distances['trasero']
            left_dist = self.distances['lateral_izq']
            right_dist = self.distances['lateral_der']
        
        # Velocidades base
        base_speed = 120
        turn_speed = 80
        slow_speed = 60
        
        # Lógica de navegación mejorada con 4 sensores
        if front_dist > OBSTACLE_THRESHOLD * 3:
            # Camino libre adelante - verificar laterales para centrado
            if left_dist < OBSTACLE_THRESHOLD and right_dist > OBSTACLE_THRESHOLD * 2:
                # Pared a la izquierda, alejarse hacia la derecha
                self.move_motors(base_speed, slow_speed)
            elif right_dist < OBSTACLE_THRESHOLD and left_dist > OBSTACLE_THRESHOLD * 2:
                # Pared a la derecha, alejarse hacia la izquierda  
                self.move_motors(slow_speed, base_speed)
            else:
                # Camino libre, avanzar
                self.move_motors(base_speed, base_speed)
                
        elif front_dist > OBSTACLE_THRESHOLD:
            # Obstáculo cerca adelante, reducir velocidad
            self.move_motors(slow_speed, slow_speed)
            
        else:
            # Obstáculo bloqueando adelante, decidir dirección
            if right_dist > left_dist and right_dist > OBSTACLE_THRESHOLD:
                # Más espacio a la derecha
                self.move_motors(turn_speed, -turn_speed)
                
            elif left_dist > OBSTACLE_THRESHOLD:
                # Girar izquierda
                self.move_motors(-turn_speed, turn_speed)
                
            elif back_dist > OBSTACLE_THRESHOLD * 2:
                # Retroceder si hay espacio atrás
                self.move_motors(-base_speed//2, -base_speed//2)
                
            else:
                # Totalmente bloqueado, girar en el lugar
                self.move_motors(-turn_speed, turn_speed)

    def simple_navigation(self):
        """Navegación reactiva simple (compatibilidad)"""
        self.advanced_navigation()

    # ========================================================================
    # LOOPS PRINCIPALES
    # ========================================================================

    def sensor_loop(self):
        """Loop de lectura de sensores"""
        print("📡 Iniciando loop de sensores...")
        
        while self.running:
            try:
                self.read_all_sensors()
                time.sleep(0.1)  # 10Hz para 4 sensores
            except Exception as e:
                print(f"Error en sensor loop: {e}")

    def fusion_loop(self):
        """Loop principal de fusión y navegación"""
        print("🧠 Iniciando loop de sensor fusion...")
        
        while self.running:
            try:
                current_time = time.time()
                dt = current_time - self.last_time
                
                # Actualizar heading con fusión
                self.update_heading(dt)
                
                # Navegación reactiva
                self.advanced_navigation()
                
                self.last_time = current_time
                time.sleep(0.05)  # 20Hz
                
            except Exception as e:
                print(f"Error en fusion loop: {e}")

    def start(self):
        """Iniciar el sistema completo"""
        self.running = True
        
        # Threads principales
        self.sensor_thread = threading.Thread(target=self.sensor_loop, daemon=True)
        self.fusion_thread = threading.Thread(target=self.fusion_loop, daemon=True)
        
        self.sensor_thread.start()
        self.fusion_thread.start()
        
        print("🚀 Sistema de sensor fusion iniciado!")
        print("Presiona Ctrl+C para detener")

    def print_status(self):
        """Imprime estado actual del robot"""
        obstacles = self.get_obstacles_global()
        
        print(f"\n--- ESTADO DEL ROBOT ---")
        print(f"📍 Posición: ({self.x:.1f}, {self.y:.1f}) cm")
        print(f"🧭 Orientación: {self.heading:.1f}°")
        print(f"⚙️ Motores: L={self.left_speed:3d}, R={self.right_speed:3d}")
        
        print(f"\n📏 Distancias de sensores:")
        sensor_status = {
            'frontal': '⬆️',
            'lateral_der': '➡️', 
            'lateral_izq': '⬅️',
            'trasero': '⬇️'
        }
        
        with self.data_lock:
            for sensor, dist in self.distances.items():
                icon = sensor_status.get(sensor, '📡')
                status = "🟢" if dist > OBSTACLE_THRESHOLD else "🔴"
                print(f"  {icon} {status} {sensor:12}: {dist:6.1f} cm")
        
        print(f"\n🚧 Obstáculos detectados: {len(obstacles)}")
        for i, obs in enumerate(obstacles[:4]):  # Mostrar hasta 4
            direction = list(sensor_status.keys())[list(sensor_status.values()).index(obs.get('sensor', '📡'))] if obs.get('sensor') in sensor_status.values() else obs.get('sensor', 'unknown')
            print(f"  {i+1}: {direction} - ({obs['x']:6.1f}, {obs['y']:6.1f}) - {obs['distance']:.1f}cm")

# ============================================================================
# NAVEGACIÓN AVANZADA (OPCIONAL)
# ============================================================================

class AdvancedNavigation(RobotController):
    """Extensión con navegación más sofisticada"""
    
    def __init__(self):
        super().__init__()
        self.obstacle_map = []  # Historia de obstáculos
        self.target_x = 0       # Objetivo X
        self.target_y = 0       # Objetivo Y
        
    def add_obstacle_to_map(self, obstacles):
        """Añade obstáculos al mapa persistente"""
        current_time = time.time()
        
        for obs in obstacles:
            # Verificar si ya existe un obstáculo cercano
            is_duplicate = False
            for existing in self.obstacle_map:
                distance = sqrt((obs['x'] - existing['x'])**2 + (obs['y'] - existing['y'])**2)
                if distance < 10:  # 10cm threshold para duplicados
                    existing['last_seen'] = current_time
                    is_duplicate = True
                    break
            
            if not is_duplicate:
                obs['last_seen'] = current_time
                self.obstacle_map.append(obs)
        
        # Remover obstáculos viejos (no vistos por >30 segundos)
        self.obstacle_map = [obs for obs in self.obstacle_map 
                            if current_time - obs['last_seen'] < 30]

    def path_planning_simple(self):
        """Navegación hacia objetivo evitando obstáculos conocidos"""
        # Vector hacia el objetivo
        dx = self.target_x - self.x
        dy = self.target_y - self.y
        
        if abs(dx) < 5 and abs(dy) < 5:  # Llegamos al objetivo
            self.stop()
            return True
        
        # Ángulo hacia el objetivo
        target_angle = degrees(atan2(dy, dx))
        angle_diff = target_angle - self.heading
        
        # Normalizar diferencia angular
        while angle_diff > 180:
            angle_diff -= 360
        while angle_diff < -180:
            angle_diff += 360
        
        # Verificar obstáculos inmediatos
        front_clear = self.distances['frontal'] > OBSTACLE_THRESHOLD * 2
        
        if front_clear and abs(angle_diff) < 30:
            # Ir directo al objetivo
            speed = 150
            self.move_motors(speed, speed)
            
        elif angle_diff > 15:
            # Girar izquierda hacia objetivo
            self.move_motors(-100, 100)
            
        elif angle_diff < -15:
            # Girar derecha hacia objetivo  
            self.move_motors(100, -100)
            
        else:
            # Usar navegación reactiva si hay obstáculos
            self.advanced_navigation()
        
        return False  # No hemos llegado

    def set_target(self, x, y):
        """Establecer objetivo de navegación"""
        self.target_x = x
        self.target_y = y
        print(f"🎯 Objetivo establecido: ({x}, {y})")

# ============================================================================
# PROGRAMA PRINCIPAL
# ============================================================================

def main():
    """Programa principal con menú de opciones"""
    
    print("🤖 SISTEMA DE SENSOR FUSION - ROBOT DE RESCATE")
    print("4 Sensores HC-SR04 + MPU6050 + Control Directo RPi")
    print("=" * 60)
    
    # Crear robot
    robot = RobotController()
    
    print("\nOpciones de ejecución:")
    print("1. Navegación automática reactiva")
    print("2. Navegación hacia objetivo específico")  
    print("3. Solo monitoreo (sin movimiento)")
    print("4. Modo debug (print continuo)")
    
    try:
        choice = input("\nSelecciona modo [1-4]: ").strip()
        
        if choice == "1":
            # Modo navegación reactiva
            print("\n🚀 Iniciando navegación automática...")
            robot.start()
            
            # Loop principal
            while True:
                robot.print_status()
                time.sleep(3)
                
        elif choice == "2":
            # Modo navegación con objetivo
            advanced_robot = AdvancedNavigation()
            
            # Pedir coordenadas objetivo
            try:
                target_x = float(input("Objetivo X (cm): "))
                target_y = float(input("Objetivo Y (cm): "))
                advanced_robot.set_target(target_x, target_y)
            except:
                print("Coordenadas inválidas, usando (100, 0)")
                advanced_robot.set_target(100, 0)
            
            print("\n🎯 Iniciando navegación hacia objetivo...")
            advanced_robot.start()
            
            # Loop con path planning
            while True:
                obstacles = advanced_robot.get_obstacles_global()
                advanced_robot.add_obstacle_to_map(obstacles)
                
                arrived = advanced_robot.path_planning_simple()
                
                if arrived:
                    print("🏆 ¡Objetivo alcanzado!")
                    break
                    
                advanced_robot.print_status()
                time.sleep(2)
                
        elif choice == "3":
            # Modo solo sensores (sin movimiento)
            print("\n📡 Modo monitoreo - Solo leyendo sensores...")
            robot.start()
            
            while True:
                robot.read_all_sensors()
                obstacles = robot.get_obstacles_global()
                
                print(f"\n🕐 {time.strftime('%H:%M:%S')}")
                sensor_icons = {'frontal': '⬆️', 'lateral_der': '➡️', 'lateral_izq': '⬅️', 'trasero': '⬇️'}
                
                for sensor, dist in robot.distances.items():
                    icon = sensor_icons.get(sensor, '📡')
                    status = "🟢 LIBRE" if dist > OBSTACLE_THRESHOLD else "🔴 BLOQUEADO"
                    print(f"  {icon} {sensor:12}: {dist:6.1f}cm {status}")
                
                if obstacles:
                    print(f"\n🚧 {len(obstacles)} obstáculos:")
                    for obs in obstacles:
                        print(f"  - {obs['sensor']}: ({obs['x']:.0f}, {obs['y']:.0f})")
                
                time.sleep(1)
                
        elif choice == "4":
            # Modo debug
            print("\n🔍 Modo debug - Información detallada...")
            robot.start()
            
            while True:
                current_time = time.time()
                dt = current_time - robot.last_time
                
                # Leer todos los sensores manualmente
                robot.read_all_sensors()
                gyro_z = robot.read_gyro_z()
                
                print(f"\n{'='*60}")
                print(f"🕐 Tiempo: {time.strftime('%H:%M:%S')} | dt: {dt:.3f}s")
                
                # Estado de sensores raw
                print(f"📡 Sensores HC-SR04 (4 sensores):")
                sensor_icons = {'frontal': '⬆️', 'lateral_der': '➡️', 'lateral_izq': '⬅️', 'trasero': '⬇️'}
                for sensor, dist in robot.distances.items():
                    icon = sensor_icons.get(sensor, '📡')
                    print(f"  {icon} {sensor:12}: {dist:7.2f}cm")
                
                print(f"🧭 Giroscopio Z: {gyro_z:8.2f}°/s")
                print(f"📍 Posición: ({robot.x:7.1f}, {robot.y:7.1f})")  
                print(f"🧭 Heading: {robot.heading:7.1f}°")
                print(f"⚙️ PWM: L={robot.left_speed:4d}, R={robot.right_speed:4d}")
                
                # Obstáculos
                obstacles = robot.get_obstacles_global()
                print(f"🚧 Obstáculos: {len(obstacles)}")
                for obs in obstacles:
                    print(f"  {obs['sensor']:12}: ({obs['x']:6.1f}, {obs['y']:6.1f}) {obs['distance']:.1f}cm")
                
                robot.last_time = current_time
                time.sleep(0.5)
        
        else:
            print("Opción inválida")
            return
            
    except KeyboardInterrupt:
        print(f"\n🛑 Deteniendo sistema...")
        
    except Exception as e:
        print(f"\n❌ Error: {e}")
        
    finally:
        # Cleanup
        robot.running = False
        robot.stop()
        
        try:
            GPIO.cleanup()
        except:
            pass
            
        print("✅ Sistema terminado correctamente")

# ============================================================================
# FUNCIONES AUXILIARES
# ============================================================================

def calibrate_speed():
    """Función para calibrar SPEED_CALIBRATION experimentalmente"""
    print("\n🔧 CALIBRACIÓN DE VELOCIDAD")
    print("Este test te ayudará a ajustar SPEED_CALIBRATION")
    
    robot = RobotController()
    
    try:
        print("\nInstrucciones:")
        print("1. Coloca el robot en línea recta")
        print("2. Mide la distancia recorrida en 5 segundos")
        print("3. Ajusta SPEED_CALIBRATION según el resultado")
        
        input("Presiona Enter para iniciar test...")
        
        test_speed = 200  # PWM
        duration = 5      # segundos
        
        print(f"\nMoviendo a PWM {test_speed} por {duration} segundos...")
        
        robot.move_motors(test_speed, test_speed)
        time.sleep(duration)
        robot.stop()
        
        print("\nTest completado!")
        
        try:
            actual_distance = float(input("Distancia real recorrida (cm): "))
            
            # Calcular factor de calibración
            expected_distance = test_speed / 255.0 * SPEED_CALIBRATION * 100 * duration
            new_calibration = SPEED_CALIBRATION * (actual_distance / expected_distance)
            
            print(f"\nResultados:")
            print(f"Factor actual: {SPEED_CALIBRATION}")
            print(f"Factor sugerido: {new_calibration:.3f}")
            print(f"Cambia SPEED_CALIBRATION = {new_calibration:.3f} en el código")
            
        except:
            print("Valor inválido")
            
    except KeyboardInterrupt:
        robot.stop()
        
    finally:
        GPIO.cleanup()

if __name__ == "__main__":
    import sys
    
    if len(sys.argv) > 1 and sys.argv[1] == "calibrate":
        calibrate_speed()
    else:
        main()