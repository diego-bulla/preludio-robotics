#!/usr/bin/env python3
# navigation.py - Sistema de Navegación de Alto Nivel

import time
import math
import threading
import numpy as np
from enum import Enum
from collections import deque
from config import *

class NavigationMode(Enum):
    """Modos de navegación disponibles"""
    IDLE = "idle"
    EXPLORE = "explore"
    AVOID_OBSTACLES = "avoid_obstacles"
    GOTO_POINT = "goto_point"
    FOLLOW_WALL = "follow_wall"
    EMERGENCY_STOP = "emergency_stop"
    COMPETITION = "competition"

class NavigationState(Enum):
    """Estados internos de navegación"""
    MOVING_FORWARD = "moving_forward"
    TURNING_LEFT = "turning_left"
    TURNING_RIGHT = "turning_right"
    BACKING_UP = "backing_up"
    STOPPED = "stopped"
    ANALYZING = "analyzing"

class PathPlanner:
    """Planificador de rutas simple"""
    
    def __init__(self):
        self.waypoints = deque()
        self.current_target = None
        self.path_tolerance = 5.0  # cm
        
    def add_waypoint(self, x, y):
        """Agregar punto de destino"""
        self.waypoints.append((x, y))
        if self.current_target is None:
            self.current_target = self.waypoints.popleft()
    
    def get_next_target(self):
        """Obtener siguiente objetivo"""
        if self.waypoints and self.current_target is None:
            self.current_target = self.waypoints.popleft()
        return self.current_target
    
    def target_reached(self, current_x, current_y):
        """Verificar si se alcanzó el objetivo"""
        if self.current_target is None:
            return False
        
        target_x, target_y = self.current_target
        distance = math.sqrt((current_x - target_x)**2 + (current_y - target_y)**2)
        
        if distance <= self.path_tolerance:
            self.current_target = None
            return True
        return False
    
    def clear_path(self):
        """Limpiar ruta actual"""
        self.waypoints.clear()
        self.current_target = None

class ObstacleAvoidance:
    """Sistema de evasión de obstáculos"""
    
    def __init__(self):
        self.avoidance_active = False
        self.avoidance_start_time = 0
        self.max_avoidance_time = 10.0  # segundos máximos en evasión
        
    def needs_avoidance(self, sensor_data):
        """Determinar si se necesita evasión de obstáculos"""
        front = sensor_data.get('front', ULTRASONIC_MAX_DISTANCE)
        left = sensor_data.get('left', ULTRASONIC_MAX_DISTANCE)
        right = sensor_data.get('right', ULTRASONIC_MAX_DISTANCE)
        
        # Obstáculo crítico al frente
        if 0 < front < DANGER_THRESHOLD_CM:
            return True, "critical_front"
        
        # Obstáculo moderado al frente
        if 0 < front < OBSTACLE_THRESHOLD_CM:
            return True, "moderate_front"
        
        # Obstáculos laterales muy cerca
        if (0 < left < DANGER_THRESHOLD_CM) or (0 < right < DANGER_THRESHOLD_CM):
            return True, "lateral_danger"
        
        return False, "clear"
    
    def get_avoidance_action(self, sensor_data, robot_state):
        """Obtener acción de evasión basada en sensores"""
        front = sensor_data.get('front', ULTRASONIC_MAX_DISTANCE)
        back = sensor_data.get('back', ULTRASONIC_MAX_DISTANCE)
        left = sensor_data.get('left', ULTRASONIC_MAX_DISTANCE)
        right = sensor_data.get('right', ULTRASONIC_MAX_DISTANCE)
        
        # Parada de emergencia si hay obstáculo muy cerca
        if front < DANGER_THRESHOLD_CM and front > 0:
            return "emergency_stop", SPEED_NORMAL
        
        # Si hay obstáculo al frente, decidir hacia dónde girar
        if front < OBSTACLE_THRESHOLD_CM and front > 0:
            if left > right and left > SAFE_DISTANCE_CM:
                return "turn_left", SPEED_TURN
            elif right > left and right > SAFE_DISTANCE_CM:
                return "turn_right", SPEED_TURN
            elif back > SAFE_DISTANCE_CM:
                return "back_up", SPEED_REVERSE
            else:
                # Rodeado de obstáculos - girar en el lugar
                return "spin_right", SPEED_TURN
        
        # Navegación normal
        return "forward", SPEED_NORMAL
    
    def start_avoidance(self):
        """Iniciar secuencia de evasión"""
        self.avoidance_active = True
        self.avoidance_start_time = time.time()
    
    def stop_avoidance(self):
        """Terminar secuencia de evasión"""
        self.avoidance_active = False
    
    def is_timeout(self):
        """Verificar si la evasión ha tomado demasiado tiempo"""
        if self.avoidance_active:
            return (time.time() - self.avoidance_start_time) > self.max_avoidance_time
        return False

class WallFollower:
    """Sistema de seguimiento de paredes"""
    
    def __init__(self, preferred_side='left', target_distance=25.0):
        self.preferred_side = preferred_side  # 'left' o 'right'
        self.target_distance = target_distance  # cm
        self.tolerance = 5.0  # cm
        self.following_active = False
        
    def start_following(self):
        """Iniciar seguimiento de pared"""
        self.following_active = True
    
    def stop_following(self):
        """Detener seguimiento de pared"""
        self.following_active = False
    
    def get_wall_following_action(self, sensor_data):
        """Obtener acción para seguir pared"""
        if not self.following_active:
            return "forward", SPEED_NORMAL
        
        side_distance = sensor_data.get(self.preferred_side, ULTRASONIC_MAX_DISTANCE)
        front_distance = sensor_data.get('front', ULTRASONIC_MAX_DISTANCE)
        
        # Si hay obstáculo al frente, girar alejándose de la pared
        if front_distance < OBSTACLE_THRESHOLD_CM and front_distance > 0:
            if self.preferred_side == 'left':
                return "turn_right", SPEED_TURN
            else:
                return "turn_left", SPEED_TURN
        
        # Ajustar distancia a la pared
        if side_distance > 0:
            distance_error = side_distance - self.target_distance
            
            if abs(distance_error) <= self.tolerance:
                # Distancia correcta - seguir adelante
                return "forward", SPEED_NORMAL
            elif distance_error > self.tolerance:
                # Muy lejos de la pared - acercarse
                if self.preferred_side == 'left':
                    return "turn_left", SPEED_TURN
                else:
                    return "turn_right", SPEED_TURN
            else:
                # Muy cerca de la pared - alejarse
                if self.preferred_side == 'left':
                    return "turn_right", SPEED_TURN
                else:
                    return "turn_left", SPEED_TURN
        
        return "forward", SPEED_NORMAL

class CompetitionNavigator:
    """Navegador específico para la competencia de robótica"""
    
    def __init__(self):
        # Zonas definidas del campo de competencia
        self.zones = {
            'start': {'x': 0, 'y': 0, 'width': FIELD_WIDTH_CM, 'height': START_ZONE_DEPTH_CM},
            'hospital': {'x': 0, 'y': 0, 'width': 200, 'height': 200},
            'refuge': {'x': FIELD_WIDTH_CM-200, 'y': 0, 'width': 200, 'height': 200},
            'debris_field': {'x': FIELD_WIDTH_CM//2-100, 'y': FIELD_HEIGHT_CM//2-100, 'width': 200, 'height': 200}
        }
        
        # Objetivos de la misión
        self.mission_state = {
            'red_blocks_found': 0,
            'green_blocks_found': 0,
            'red_blocks_rescued': 0,
            'green_blocks_evacuated': 0,
            'current_mission': 'search_and_rescue'
        }
        
        # Estrategia de búsqueda
        self.search_pattern = 'spiral'  # 'spiral', 'grid', 'wall_follow'
        
    def get_competition_action(self, robot_state, sensor_data, time_remaining):
        """Obtener acción específica para la competencia"""
        current_zone = self.identify_current_zone(robot_state['x'], robot_state['y'])
        
        # Estrategia basada en el tiempo restante
        if time_remaining < 30:  # Últimos 30 segundos
            return self.emergency_strategy(robot_state, current_zone)
        elif time_remaining < 60:  # Último minuto
            return self.evacuation_strategy(robot_state, current_zone)
        else:  # Tiempo normal
            return self.search_strategy(robot_state, sensor_data, current_zone)
    
    def identify_current_zone(self, x, y):
        """Identificar en qué zona se encuentra el robot"""
        for zone_name, zone_info in self.zones.items():
            if (zone_info['x'] <= x <= zone_info['x'] + zone_info['width'] and
                zone_info['y'] <= y <= zone_info['y'] + zone_info['height']):
                return zone_name
        return 'unknown'
    
    def search_strategy(self, robot_state, sensor_data, current_zone):
        """Estrategia de búsqueda de bloques"""
        # Priorizar búsqueda en campo de escombros
        if current_zone != 'debris_field':
            # Navegar hacia el campo de escombros
            target_x = self.zones['debris_field']['x'] + self.zones['debris_field']['width']//2
            target_y = self.zones['debris_field']['y'] + self.zones['debris_field']['height']//2
            return self.navigate_to_point(robot_state, target_x, target_y)
        else:
            # Explorar sistemáticamente el área
            return self.systematic_search(robot_state, sensor_data)
    
    def evacuation_strategy(self, robot_state, current_zone):
        """Estrategia de evacuación de bloques"""
        if self.mission_state['red_blocks_found'] > self.mission_state['red_blocks_rescued']:
            # Llevar bloques rojos al hospital
            target_x = self.zones['hospital']['x'] + self.zones['hospital']['width']//2
            target_y = self.zones['hospital']['y'] + self.zones['hospital']['height']//2
            return self.navigate_to_point(robot_state, target_x, target_y)
        elif self.mission_state['green_blocks_found'] > self.mission_state['green_blocks_evacuated']:
            # Llevar bloques verdes al refugio
            target_x = self.zones['refuge']['x'] + self.zones['refuge']['width']//2
            target_y = self.zones['refuge']['y'] + self.zones['refuge']['height']//2
            return self.navigate_to_point(robot_state, target_x, target_y)
        else:
            return "explore", SPEED_NORMAL
    
    def emergency_strategy(self, robot_state, current_zone):
        """Estrategia de emergencia para los últimos segundos"""
        # Asegurar que todos los bloques estén en sus zonas correctas
        return "stop", 0
    
    def navigate_to_point(self, robot_state, target_x, target_y):
        """Navegar hacia un punto específico"""
        current_x = robot_state['x']
        current_y = robot_state['y']
        current_theta = robot_state['theta']
        
        # Calcular dirección hacia el objetivo
        dx = target_x - current_x
        dy = target_y - current_y
        target_angle = math.atan2(dy, dx)
        
        # Diferencia angular
        angle_diff = target_angle - current_theta
        angle_diff = math.atan2(math.sin(angle_diff), math.cos(angle_diff))  # Normalizar a [-π, π]
        
        # Decidir acción basada en diferencia angular
        if abs(angle_diff) > math.radians(15):  # Más de 15 grados de diferencia
            if angle_diff > 0:
                return "turn_left", SPEED_TURN
            else:
                return "turn_right", SPEED_TURN
        else:
            # Avanzar hacia el objetivo
            return "forward", SPEED_NORMAL
    
    def systematic_search(self, robot_state, sensor_data):
        """Búsqueda sistemática en espiral"""
        # Implementación simple de búsqueda en espiral
        front_clear = sensor_data.get('front', ULTRASONIC_MAX_DISTANCE) > OBSTACLE_THRESHOLD_CM
        
        if front_clear:
            return "forward", SPEED_SLOW
        else:
            return "turn_right", SPEED_TURN

class RobotNavigator:
    """Navegador principal del robot"""
    
    def __init__(self, motor_controller, sensor_fusion):
        print("🧭 Inicializando Sistema de Navegación...")
        
        # Referencias a otros sistemas
        self.motors = motor_controller
        self.sensor_fusion = sensor_fusion
        
        # Componentes de navegación
        self.path_planner = PathPlanner()
        self.obstacle_avoidance = ObstacleAvoidance()
        self.wall_follower = WallFollower()
        self.competition_nav = CompetitionNavigator()
        
        # Estado del navegador
        self.current_mode = NavigationMode.IDLE
        self.current_state = NavigationState.STOPPED
        self.running = False
        
        # Thread de navegación
        self.nav_thread = None
        self.navigation_lock = threading.Lock()
        
        # Variables de control
        self.target_speed = 0
        self.last_action = "stop"
        self.action_start_time = 0
        self.min_action_duration = 0.5  # Duración mínima de cada acción
        
        # Métricas de navegación
        self.metrics = {
            'distance_traveled': 0.0,
            'obstacles_avoided': 0,
            'mode_changes': 0,
            'navigation_errors': 0,
            'uptime': 0.0
        }
        
        self.start_time = time.time()
        
        print("✅ Sistema de Navegación inicializado")
    
    def start(self):
        """Iniciar sistema de navegación"""
        if not self.running:
            self.running = True
            self.nav_thread = threading.Thread(target=self._navigation_loop, daemon=True)
            self.nav_thread.start()
            print("🔄 Navegación iniciada")
    
    def stop(self):
        """Detener sistema de navegación"""
        self.running = False
        if self.nav_thread and self.nav_thread.is_alive():
            self.nav_thread.join(timeout=2)
        self.motors.stop_motors()
        print("🛑 Navegación detenida")
    
    def set_mode(self, mode):
        """Cambiar modo de navegación"""
        with self.navigation_lock:
            if self.current_mode != mode:
                print(f"🔄 Cambiando modo: {self.current_mode.value} → {mode.value}")
                self.current_mode = mode
                self.metrics['mode_changes'] += 1
                
                # Limpiar estado según el modo
                if mode == NavigationMode.IDLE:
                    self.motors.stop_motors()
                elif mode == NavigationMode.GOTO_POINT:
                    # El path_planner ya debe tener waypoints configurados
                    pass
                elif mode == NavigationMode.FOLLOW_WALL:
                    self.wall_follower.start_following()
                elif mode == NavigationMode.EXPLORE:
                    self.wall_follower.stop_following()
    
    def add_waypoint(self, x, y):
        """Agregar punto de destino"""
        self.path_planner.add_waypoint(x, y)
        if self.current_mode == NavigationMode.IDLE:
            self.set_mode(NavigationMode.GOTO_POINT)
    
    def emergency_stop(self):
        """Parada de emergencia"""
        self.set_mode(NavigationMode.EMERGENCY_STOP)
        self.motors.stop_motors()
        print("🚨 PARADA DE EMERGENCIA ACTIVADA")
    
    def _navigation_loop(self):
        """Loop principal de navegación"""
        last_update = time.time()
        
        while self.running:
            try:
                current_time = time.time()
                dt = current_time - last_update
                
                if dt >= (1.0 / AI_UPDATE_FREQUENCY):
                    self._update_navigation()
                    last_update = current_time
                
                # Actualizar métricas
                self.metrics['uptime'] = current_time - self.start_time
                
                time.sleep(0.05)  # 20Hz loop base
                
            except Exception as e:
                print(f"❌ Error en loop de navegación: {e}")
                self.metrics['navigation_errors'] += 1
                time.sleep(0.1)
    
    def _update_navigation(self):
        """Actualizar navegación basada en el modo actual"""
        with self.navigation_lock:
            # Obtener estado actual
            robot_state = self.sensor_fusion.get_robot_state()
            sensor_data = self.sensor_fusion.get_latest_sensors()
            
            # Verificar condiciones de emergencia
            if self._check_emergency_conditions(sensor_data):
                if self.current_mode != NavigationMode.EMERGENCY_STOP:
                    self.emergency_stop()
                return
            
            # Ejecutar lógica según el modo
            action, speed = self._get_navigation_action(robot_state, sensor_data)
            
            # Aplicar acción si ha pasado tiempo suficiente desde la última
            if time.time() - self.action_start_time >= self.min_action_duration:
                self._execute_action(action, speed)
                self.last_action = action
                self.action_start_time = time.time()
    
    def _check_emergency_conditions(self, sensor_data):
        """Verificar condiciones de parada de emergencia"""
        # Obstáculo muy cerca en cualquier dirección
        for distance in sensor_data.values():
            if isinstance(distance, (int, float)) and 0 < distance < DANGER_THRESHOLD_CM:
                return True
        
        # Timeout de evasión de obstáculos
        if self.obstacle_avoidance.is_timeout():
            print("⚠️  Timeout en evasión de obstáculos")
            return True
        
        return False
    
    def _get_navigation_action(self, robot_state, sensor_data):
        """Obtener acción de navegación basada en el modo actual"""
        
        if self.current_mode == NavigationMode.IDLE:
            return "stop", 0
        
        elif self.current_mode == NavigationMode.EMERGENCY_STOP:
            return "stop", 0
        
        elif self.current_mode == NavigationMode.AVOID_OBSTACLES:
            needs_avoidance, reason = self.obstacle_avoidance.needs_avoidance(sensor_data)
            
            if needs_avoidance:
                if not self.obstacle_avoidance.avoidance_active:
                    self.obstacle_avoidance.start_avoidance()
                    self.metrics['obstacles_avoided'] += 1
                
                return self.obstacle_avoidance.get_avoidance_action(sensor_data, robot_state)
            else:
                self.obstacle_avoidance.stop_avoidance()
                self.set_mode(NavigationMode.EXPLORE)
                return "forward", SPEED_NORMAL
        
        elif self.current_mode == NavigationMode.GOTO_POINT:
            # Verificar si se alcanzó el objetivo
            if self.path_planner.target_reached(robot_state['x'], robot_state['y']):
                if not self.path_planner.get_next_target():
                    self.set_mode(NavigationMode.IDLE)
                    return "stop", 0
            
            # Verificar si hay obstáculos en el camino
            needs_avoidance, _ = self.obstacle_avoidance.needs_avoidance(sensor_data)
            if needs_avoidance:
                self.set_mode(NavigationMode.AVOID_OBSTACLES)
                return self.obstacle_avoidance.get_avoidance_action(sensor_data, robot_state)
            
            # Navegar hacia el punto objetivo
            target = self.path_planner.get_next_target()
            if target:
                return self.competition_nav.navigate_to_point(robot_state, target[0], target[1])
            else:
                return "stop", 0
        
        elif self.current_mode == NavigationMode.FOLLOW_WALL:
            return self.wall_follower.get_wall_following_action(sensor_data)
        
        elif self.current_mode == NavigationMode.EXPLORE:
            # Verificar si hay obstáculos
            needs_avoidance, _ = self.obstacle_avoidance.needs_avoidance(sensor_data)
            if needs_avoidance:
                self.set_mode(NavigationMode.AVOID_OBSTACLES)
                return self.obstacle_avoidance.get_avoidance_action(sensor_data, robot_state)
            
            # Usar IA para exploración
            decision = self.sensor_fusion.get_navigation_decision()
            return self._map_ai_decision_to_action(decision)
        
        elif self.current_mode == NavigationMode.COMPETITION:
            # Lógica específica de competencia
            time_remaining = GAME_DURATION - self.metrics['uptime']
            return self.competition_nav.get_competition_action(robot_state, sensor_data, time_remaining)
        
        # Modo desconocido
        return "stop", 0
    
    def _map_ai_decision_to_action(self, decision):
        """Mapear decisión de IA a acción de motor"""
        action_map = {
            'forward': ('forward', SPEED_NORMAL),
            'backward': ('backward', SPEED_REVERSE),
            'left': ('turn_left', SPEED_TURN),
            'right': ('turn_right', SPEED_TURN)
        }
        
        action = decision.get('action', 'forward')
        return action_map.get(action, ('stop', 0))
    
    def _execute_action(self, action, speed):
        """Ejecutar acción de movimiento"""
        try:
            if action == "forward":
                self.motors.move_forward(speed)
                self.current_state = NavigationState.MOVING_FORWARD
            elif action == "backward" or action == "back_up":
                self.motors.move_backward(speed)
                self.current_state = NavigationState.BACKING_UP
            elif action == "turn_left":
                self.motors.turn_left(speed)
                self.current_state = NavigationState.TURNING_LEFT
            elif action == "turn_right":
                self.motors.turn_right(speed)
                self.current_state = NavigationState.TURNING_RIGHT
            elif action == "spin_left":
                self.motors.spin_left(speed)
                self.current_state = NavigationState.TURNING_LEFT
            elif action == "spin_right":
                self.motors.spin_right(speed)
                self.current_state = NavigationState.TURNING_RIGHT
            elif action == "stop" or action == "emergency_stop":
                self.motors.stop_motors()
                self.current_state = NavigationState.STOPPED
            
            # Actualizar métricas
            if action in ["forward", "backward"]:
                # Estimar distancia basada en velocidad y tiempo
                estimated_distance = (speed / 100.0) * 5.0 * self.min_action_duration  # Estimación simple
                self.metrics['distance_traveled'] += estimated_distance
                
        except Exception as e:
            print(f"❌ Error ejecutando acción {action}: {e}")
            self.motors.stop_motors()
            self.metrics['navigation_errors'] += 1
    
    def get_status(self):
        """Obtener estado actual del navegador"""
        with self.navigation_lock:
            return {
                'mode': self.current_mode.value,
                'state': self.current_state.value,
                'last_action': self.last_action,
                'target_speed': self.target_speed,
                'running': self.running,
                'metrics': self.metrics.copy()
            }
    
    def get_mission_status(self):
        """Obtener estado de la misión de competencia"""
        return self.competition_nav.mission_state.copy()
    
    def reset_metrics(self):
        """Reiniciar métricas de navegación"""
        self.metrics = {
            'distance_traveled': 0.0,
            'obstacles_avoided': 0,
            'mode_changes': 0,
            'navigation_errors': 0,
            'uptime': 0.0
        }
        self.start_time = time.time()
        print("🔄 Métricas de navegación reiniciadas")
    
    def cleanup(self):
        """Limpiar recursos"""
        self.stop()
        print("🧹 Navegador limpiado")

if __name__ == "__main__":
    # Prueba básica del sistema de navegación
    print("🧪 Probando Sistema de Navegación...")
    
    # Mock objects para prueba
    class MockMotors:
        def move_forward(self, speed): print(f"🔄 Adelante {speed}%")
        def move_backward(self, speed): print(f"🔄 Atrás {speed}%")
        def turn_left(self, speed): print(f"🔄 Izquierda {speed}%")
        def turn_right(self, speed): print(f"🔄 Derecha {speed}%")
        def spin_left(self, speed): print(f"🔄 Spin izq {speed}%")
        def spin_right(self, speed): print(f"🔄 Spin der {speed}%")
        def stop_motors(self): print("🛑 Parar")
    
    class MockSensorFusion:
        def get_robot_state(self):
            return {'x': 0, 'y': 0, 'theta': 0, 'velocity': 0}
        def get_latest_sensors(self):
            return {'front': 100, 'back': 200, 'left': 150, 'right': 120}
        def get_navigation_decision(self):
            return {'action': 'forward', 'confidence': 0.8}
    
    try:
        navigator = RobotNavigator(MockMotors(), MockSensorFusion())
        
        # Probar diferentes modos
        print("\n🧪 Probando modos de navegación:")
        
        navigator.start()
        
        # Modo exploración
        navigator.set_mode(NavigationMode.EXPLORE)
        time.sleep(2)
        
        # Modo goto punto
        navigator.add_waypoint(50, 50)
        time.sleep(2)
        
        # Modo seguir pared
        navigator.set_mode(NavigationMode.FOLLOW_WALL)
        time.sleep(2)
        
        # Mostrar estado
        status = navigator.get_status()
        print(f"\n📊 Estado final:")
        for key, value in status.items():
            print(f"   {key}: {value}")
        
        navigator.cleanup()
        
    except KeyboardInterrupt:
        print("\n❌ Prueba interrumpida")
        navigator.cleanup()
    except Exception as e:
        print(f"❌ Error: {e}")
        navigator.cleanup()