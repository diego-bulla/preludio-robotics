#!/usr/bin/env python3
# sensor_fusion.py - Fusión de Sensores e IA para Navegación

import numpy as np
import time
import threading
import math
import cv2
from collections import deque
from config import *

class KalmanFilter:
    """Filtro de Kalman simple para estimación de posición"""
    
    def __init__(self, initial_x=0, initial_y=0, initial_theta=0):
        # Estado: [x, y, theta, vx, vy, vtheta]
        self.state = np.array([initial_x, initial_y, initial_theta, 0, 0, 0], dtype=float)
        
        # Matriz de covarianza del estado
        self.P = np.eye(6) * KALMAN_INITIAL_UNCERTAINTY
        
        # Matriz de transición de estado (movimiento)
        self.F = np.eye(6)
        
        # Matriz de ruido del proceso
        self.Q = np.eye(6) * KALMAN_PROCESS_NOISE
        
        # Matriz de observación (medimos posición)
        self.H = np.array([
            [1, 0, 0, 0, 0, 0],  # x
            [1, 0, 0, 0, 0, 0],  # y
            [0, 0, 1, 0, 0, 0]   # theta
        ])
        
        # Matriz de ruido de medición
        self.R = np.eye(3) * KALMAN_MEASUREMENT_NOISE
        
        print("✅ Filtro de Kalman inicializado")
    
    def predict(self, dt, velocity, angular_velocity):
        """Paso de predicción basado en odometría"""
        # Actualizar matriz de transición con dt
        self.F[0, 3] = dt  # x += vx * dt
        self.F[1, 4] = dt  # y += vy * dt
        self.F[2, 5] = dt  # theta += vtheta * dt
        
        # Actualizar velocidades basadas en comandos de motor
        theta = self.state[2]
        self.state[3] = velocity * np.cos(theta)  # vx
        self.state[4] = velocity * np.sin(theta)  # vy
        self.state[5] = angular_velocity          # vtheta
        
        # Predicción del estado
        self.state = self.F @ self.state
        
        # Predicción de la covarianza
        self.P = self.F @ self.P @ self.F.T + self.Q
    
    def update(self, measurement):
        """Paso de actualización con medición externa"""
        # Calcular residuo
        y = measurement - self.H @ self.state
        
        # Matriz de covarianza del residuo
        S = self.H @ self.P @ self.H.T + self.R
        
        # Ganancia de Kalman
        K = self.P @ self.H.T @ np.linalg.inv(S)
        
        # Actualizar estado
        self.state = self.state + K @ y
        
        # Actualizar covarianza
        I = np.eye(len(self.state))
        self.P = (I - K @ self.H) @ self.P
    
    def get_position(self):
        """Obtener posición estimada [x, y, theta]"""
        return self.state[:3].copy()
    
    def get_velocity(self):
        """Obtener velocidad estimada [vx, vy, vtheta]"""
        return self.state[3:].copy()

class ObstacleMap:
    """Mapa simple de obstáculos basado en sensores"""
    
    def __init__(self, map_size=200, resolution=1.0):
        self.map_size = map_size  # Tamaño del mapa en cm
        self.resolution = resolution  # cm por píxel
        self.grid_size = int(map_size / resolution)
        
        # Mapa de ocupación: 0=libre, 1=obstáculo, 0.5=desconocido
        self.grid = np.ones((self.grid_size, self.grid_size)) * 0.5
        
        # Centro del mapa (posición inicial del robot)
        self.center_x = self.grid_size // 2
        self.center_y = self.grid_size // 2
        
        print("✅ Mapa de obstáculos inicializado")
    
    def world_to_grid(self, x, y):
        """Convertir coordenadas del mundo a coordenadas de grid"""
        grid_x = int(self.center_x + x / self.resolution)
        grid_y = int(self.center_y + y / self.resolution)
        return grid_x, grid_y
    
    def update_from_sensors(self, robot_x, robot_y, robot_theta, sensor_data):
        """Actualizar mapa basado en lecturas de sensores"""
        for sensor_name, distance in sensor_data.items():
            if distance <= 0 or distance > ULTRASONIC_MAX_DISTANCE:
                continue
            
            # Obtener posición y ángulo del sensor
            sensor_pos = SENSOR_POSITIONS.get(sensor_name)
            if not sensor_pos:
                continue
            
            sensor_angle = math.radians(sensor_pos['angle']) + robot_theta
            
            # Calcular posición del obstáculo
            obstacle_x = robot_x + distance * math.cos(sensor_angle)
            obstacle_y = robot_y + distance * math.sin(sensor_angle)
            
            # Convertir a coordenadas de grid
            grid_x, grid_y = self.world_to_grid(obstacle_x, obstacle_y)
            
            # Marcar obstáculo si está dentro del mapa
            if 0 <= grid_x < self.grid_size and 0 <= grid_y < self.grid_size:
                self.grid[grid_y, grid_x] = 1.0
            
            # Marcar espacio libre entre robot y obstáculo
            robot_grid_x, robot_grid_y = self.world_to_grid(robot_x, robot_y)
            self._mark_free_space(robot_grid_x, robot_grid_y, grid_x, grid_y)
    
    def _mark_free_space(self, x0, y0, x1, y1):
        """Marcar espacio libre usando algoritmo de línea de Bresenham"""
        points = self._bresenham_line(x0, y0, x1, y1)
        
        for x, y in points[:-1]:  # Excluir el último punto (obstáculo)
            if 0 <= x < self.grid_size and 0 <= y < self.grid_size:
                if self.grid[y, x] == 0.5:  # Solo actualizar si es desconocido
                    self.grid[y, x] = 0.0
    
    def _bresenham_line(self, x0, y0, x1, y1):
        """Algoritmo de Bresenham para generar puntos de línea"""
        points = []
        dx = abs(x1 - x0)
        dy = abs(y1 - y0)
        sx = 1 if x0 < x1 else -1
        sy = 1 if y0 < y1 else -1
        err = dx - dy
        
        x, y = x0, y0
        
        while True:
            points.append((x, y))
            
            if x == x1 and y == y1:
                break
            
            e2 = 2 * err
            if e2 > -dy:
                err -= dy
                x += sx
            if e2 < dx:
                err += dx
                y += sy
        
        return points
    
    def is_obstacle(self, x, y, safety_margin=5):
        """Verificar si hay obstáculo en una posición (con margen de seguridad)"""
        grid_x, grid_y = self.world_to_grid(x, y)
        
        # Verificar área alrededor de la posición
        margin_cells = int(safety_margin / self.resolution)
        
        for dx in range(-margin_cells, margin_cells + 1):
            for dy in range(-margin_cells, margin_cells + 1):
                check_x = grid_x + dx
                check_y = grid_y + dy
                
                if 0 <= check_x < self.grid_size and 0 <= check_y < self.grid_size:
                    if self.grid[check_y, check_x] > 0.7:  # Umbral de obstáculo
                        return True
        
        return False
    
    def get_map_image(self):
        """Obtener imagen del mapa para visualización"""
        # Convertir a imagen de 8 bits
        map_image = (self.grid * 255).astype(np.uint8)
        
        # Invertir colores (negro=obstáculo, blanco=libre)
        map_image = 255 - map_image
        
        # Convertir a color para poder dibujar el robot
        map_image = cv2.cvtColor(map_image, cv2.COLOR_GRAY2BGR)
        
        return map_image

class SimplifiedNeuralNetwork:
    """Red neuronal simplificada para decisiones de navegación"""
    
    def __init__(self):
        # Red simple: 8 entradas -> 6 ocultas -> 4 salidas
        # Entradas: 4 sensores + velocidad actual + posición relativa (x,y) + orientación
        # Salidas: probabilidades de [adelante, atrás, izquierda, derecha]
        
        self.input_size = 8
        self.hidden_size = 6
        self.output_size = 4
        
        # Inicializar pesos aleatoriamente
        self.W1 = np.random.randn(self.input_size, self.hidden_size) * 0.5
        self.b1 = np.zeros((1, self.hidden_size))
        self.W2 = np.random.randn(self.hidden_size, self.output_size) * 0.5
        self.b2 = np.zeros((1, self.output_size))
        
        # Parámetros de entrenamiento simple
        self.learning_rate = 0.01
        
        print("✅ Red neuronal inicializada")
    
    def sigmoid(self, x):
        """Función de activación sigmoide"""
        return 1 / (1 + np.exp(-np.clip(x, -500, 500)))
    
    def softmax(self, x):
        """Función softmax para probabilidades"""
        exp_x = np.exp(x - np.max(x))
        return exp_x / np.sum(exp_x)
    
    def forward(self, inputs):
        """Propagación hacia adelante"""
        # Capa oculta
        z1 = np.dot(inputs, self.W1) + self.b1
        a1 = self.sigmoid(z1)
        
        # Capa de salida
        z2 = np.dot(a1, self.W2) + self.b2
        a2 = self.softmax(z2.flatten())
        
        return a2
    
    def simple_train(self, inputs, target_action, reward):
        """Entrenamiento simple basado en recompensas"""
        # Forward pass
        output = self.forward(inputs)
        
        # Crear target vector
        target = np.zeros(self.output_size)
        target[target_action] = 1.0
        
        # Error ponderado por recompensa
        error = (target - output) * reward
        
        # Actualización simple de pesos (solo capa de salida)
        self.W2 += self.learning_rate * error.reshape(-1, 1).T
        
        return np.max(output)

class SensorFusion:
    """Clase principal para fusión de sensores e IA"""
    
    def __init__(self):
        print("🧠 Inicializando Sistema de Sensor Fusion...")
        
        # Componentes principales
        self.kalman = KalmanFilter()
        self.obstacle_map = ObstacleMap()
        self.neural_net = SimplifiedNeuralNetwork()
        
        # Estado del robot
        self.robot_state = {
            'x': 0.0,
            'y': 0.0,
            'theta': 0.0,
            'velocity': 0.0,
            'angular_velocity': 0.0,
            'last_update': time.time()
        }
        
        # Historial de datos para análisis
        self.sensor_history = deque(maxlen=100)
        self.decision_history = deque(maxlen=50)
        
        # Variables de control
        self.running = False
        self.fusion_thread = None
        self.data_lock = threading.Lock()
        
        # Métricas de rendimiento
        self.metrics = {
            'fusion_rate': 0,
            'decisions_made': 0,
            'obstacles_detected': 0,
            'safe_distance_maintained': 0
        }
        
        print("✅ Sistema de Sensor Fusion inicializado")
    
    def start(self):
        """Iniciar procesamiento de sensor fusion"""
        if not self.running:
            self.running = True
            self.fusion_thread = threading.Thread(target=self._fusion_loop, daemon=True)
            self.fusion_thread.start()
            print("🔄 Sensor fusion iniciado")
    
    def stop(self):
        """Detener procesamiento"""
        self.running = False
        if self.fusion_thread and self.fusion_thread.is_alive():
            self.fusion_thread.join(timeout=2)
        print("🛑 Sensor fusion detenido")
    
    def update_odometry(self, left_distance, right_distance, dt):
        """Actualizar odometría basada en encoders"""
        with self.data_lock:
            # Calcular desplazamiento del centro del robot
            center_distance = (left_distance + right_distance) / 2.0
            
            # Calcular rotación
            distance_diff = right_distance - left_distance
            delta_theta = distance_diff / WHEELBASE_CM
            
            # Calcular velocidades
            velocity = center_distance / dt if dt > 0 else 0
            angular_velocity = delta_theta / dt if dt > 0 else 0
            
            # Actualizar estado
            old_theta = self.robot_state['theta']
            self.robot_state['theta'] += delta_theta
            self.robot_state['x'] += center_distance * math.cos(old_theta + delta_theta/2)
            self.robot_state['y'] += center_distance * math.sin(old_theta + delta_theta/2)
            self.robot_state['velocity'] = velocity
            self.robot_state['angular_velocity'] = angular_velocity
            self.robot_state['last_update'] = time.time()
            
            # Actualizar filtro de Kalman
            self.kalman.predict(dt, velocity, angular_velocity)
    
    def update_sensors(self, sensor_data):
        """Actualizar con datos de sensores ultrasónicos"""
        with self.data_lock:
            # Agregar timestamp a los datos
            sensor_data['timestamp'] = time.time()
            self.sensor_history.append(sensor_data.copy())
            
            # Actualizar mapa de obstáculos
            self.obstacle_map.update_from_sensors(
                self.robot_state['x'],
                self.robot_state['y'],
                self.robot_state['theta'],
                {
                    'front': sensor_data.get('front', 0),
                    'back': sensor_data.get('back', 0),
                    'left': sensor_data.get('left', 0),
                    'right': sensor_data.get('right', 0)
                }
            )
            
            # Contar obstáculos detectados
            for distance in sensor_data.values():
                if isinstance(distance, (int, float)) and 0 < distance < OBSTACLE_THRESHOLD_CM:
                    self.metrics['obstacles_detected'] += 1
                    break
    
    def get_navigation_decision(self):
        """Obtener decisión de navegación usando IA"""
        with self.data_lock:
            # Preparar datos para la red neuronal
            latest_sensors = self.sensor_history[-1] if self.sensor_history else {}
            
            # Normalizar datos de entrada (0-1)
            inputs = np.array([
                min(latest_sensors.get('front', ULTRASONIC_MAX_DISTANCE) / ULTRASONIC_MAX_DISTANCE, 1.0),
                min(latest_sensors.get('back', ULTRASONIC_MAX_DISTANCE) / ULTRASONIC_MAX_DISTANCE, 1.0),
                min(latest_sensors.get('left', ULTRASONIC_MAX_DISTANCE) / ULTRASONIC_MAX_DISTANCE, 1.0),
                min(latest_sensors.get('right', ULTRASONIC_MAX_DISTANCE) / ULTRASONIC_MAX_DISTANCE, 1.0),
                min(abs(self.robot_state['velocity']) / 50.0, 1.0),  # Velocidad normalizada
                (self.robot_state['x'] % 100) / 100.0,  # Posición x mod 100
                (self.robot_state['y'] % 100) / 100.0,  # Posición y mod 100
                (self.robot_state['theta'] % (2*math.pi)) / (2*math.pi)  # Orientación normalizada
            ])
            
            # Obtener decisión de la red neuronal
            decision_probs = self.neural_net.forward(inputs)
            decision = np.argmax(decision_probs)
            confidence = np.max(decision_probs)
            
            # Mapear decisión a acción
            actions = ['forward', 'backward', 'left', 'right']
            action = actions[decision]
            
            # Aplicar reglas de seguridad
            action = self._apply_safety_rules(action, latest_sensors)
            
            # Registrar decisión
            decision_info = {
                'action': action,
                'confidence': confidence,
                'inputs': inputs,
                'sensor_data': latest_sensors,
                'timestamp': time.time()
            }
            
            self.decision_history.append(decision_info)
            self.metrics['decisions_made'] += 1
            
            return decision_info
    
    def _apply_safety_rules(self, action, sensor_data):
        """Aplicar reglas de seguridad sobre la decisión de IA"""
        # Regla 1: No avanzar si hay obstáculo muy cerca al frente
        if action == 'forward' and sensor_data.get('front', ULTRASONIC_MAX_DISTANCE) < DANGER_THRESHOLD_CM:
            return 'backward'
        
        # Regla 2: No retroceder si hay obstáculo muy cerca atrás
        if action == 'backward' and sensor_data.get('back', ULTRASONIC_MAX_DISTANCE) < DANGER_THRESHOLD_CM:
            return 'forward'
        
        # Regla 3: No girar hacia un lado si hay obstáculo muy cerca
        if action == 'left' and sensor_data.get('left', ULTRASONIC_MAX_DISTANCE) < DANGER_THRESHOLD_CM:
            return 'right'
        
        if action == 'right' and sensor_data.get('right', ULTRASONIC_MAX_DISTANCE) < DANGER_THRESHOLD_CM:
            return 'left'
        
        # Regla 4: Verificar distancia de seguridad mantenida
        min_distance = min([d for d in sensor_data.values() if isinstance(d, (int, float)) and d > 0])
        if min_distance > SAFE_DISTANCE_CM:
            self.metrics['safe_distance_maintained'] += 1
        
        return action
    
    def simple_learning(self, action_taken, outcome_reward):
        """Aprendizaje simple basado en recompensas"""
        if self.decision_history:
            last_decision = self.decision_history[-1]
            action_map = {'forward': 0, 'backward': 1, 'left': 2, 'right': 3}
            
            if action_taken in action_map:
                self.neural_net.simple_train(
                    last_decision['inputs'],
                    action_map[action_taken],
                    outcome_reward
                )
    
    def _fusion_loop(self):
        """Loop principal de fusión de sensores"""
        last_time = time.time()
        fusion_count = 0
        
        while self.running:
            current_time = time.time()
            dt = current_time - last_time
            
            if dt >= (1.0 / SENSOR_FUSION_FREQUENCY):
                # Aquí se podría hacer procesamiento adicional
                # como filtrado de datos, detección de patrones, etc.
                
                fusion_count += 1
                self.metrics['fusion_rate'] = fusion_count / (current_time - time.time() + 1)
                
                last_time = current_time
            
            time.sleep(0.01)  # 100Hz loop base
    
    def get_robot_state(self):
        """Obtener estado actual del robot"""
        with self.data_lock:
            return self.robot_state.copy()
    
    def get_latest_sensors(self):
        """Obtener últimos datos de sensores"""
        with self.data_lock:
            return self.sensor_history[-1].copy() if self.sensor_history else {}
    
    def get_obstacle_map(self):
        """Obtener mapa de obstáculos para visualización"""
        return self.obstacle_map.get_map_image()
    
    def get_metrics(self):
        """Obtener métricas de rendimiento"""
        return self.metrics.copy()
    
    def reset_position(self):
        """Resetear posición del robot"""
        with self.data_lock:
            self.robot_state['x'] = 0.0
            self.robot_state['y'] = 0.0
            self.robot_state['theta'] = 0.0
            self.kalman = KalmanFilter()
            print("🔄 Posición del robot reseteada")
    
    def cleanup(self):
        """Limpiar recursos"""
        self.stop()
        print("🧹 Sensor fusion limpiado")

if __name__ == "__main__":
    # Prueba básica del sistema de sensor fusion
    try:
        print("🧪 Probando Sistema de Sensor Fusion...")
        
        fusion = SensorFusion()
        fusion.start()
        
        # Simular datos de prueba
        for i in range(50):
            # Simular odometría
            fusion.update_odometry(1.0, 1.0, 0.1)  # 1cm por cada rueda, 0.1s
            
            # Simular sensores
            sensor_data = {
                'front': 100 + i,
                'back': 200,
                'left': 150,
                'right': 120
            }
            fusion.update_sensors(sensor_data)
            
            # Obtener decisión
            decision = fusion.get_navigation_decision()
            print(f"Decisión {i}: {decision['action']} (confianza: {decision['confidence']:.2f})")
            
            # Simular aprendizaje
            reward = 1.0 if decision['action'] == 'forward' else 0.5
            fusion.simple_learning(decision['action'], reward)
            
            time.sleep(0.1)
        
        # Mostrar métricas
        metrics = fusion.get_metrics()
        print(f"\n📊 Métricas:")
        for key, value in metrics.items():
            print(f"   {key}: {value}")
        
        fusion.cleanup()
        
    except KeyboardInterrupt:
        print("\n❌ Prueba interrumpida")
        fusion.cleanup()
    except Exception as e:
        print(f"❌ Error: {e}")
        fusion.cleanup()