#!/usr/bin/env python3
# main.py - Programa Principal del Robot Autónomo

import sys
import time
import signal
import argparse
import threading
from datetime import datetime
import cv2
import numpy as np

# Importar módulos del robot
from config import *
from motor_control import MotorController
from arduino_comm import ArduinoComm
from sensor_fusion import SensorFusion
from navigation import RobotNavigator, NavigationMode

class RobotSystem:
    """Sistema principal del robot autónomo"""
    
    def __init__(self, enable_camera=True, enable_arduino=True):
        print("🤖 === ROBOT AUTÓNOMO INICIANDO ===")
        print(f"Versión: {VERSION}")
        print(f"Timestamp: {datetime.now().strftime('%Y-%m-%d %H:%M:%S')}")
        print("=" * 50)
        
        # Configurar manejo de señales
        signal.signal(signal.SIGINT, self.signal_handler)
        signal.signal(signal.SIGTERM, self.signal_handler)
        
        # Estado del sistema
        self.running = False
        self.initialized = False
        
        # Componentes del robot
        self.motors = None
        self.arduino = None
        self.sensor_fusion = None
        self.navigator = None
        self.camera = None
        
        # Configuración
        self.enable_camera = enable_camera and CAMERA_ENABLED
        self.enable_arduino = enable_arduino
        
        # Threads de monitoreo
        self.monitor_thread = None
        self.camera_thread = None
        
        # Métricas del sistema
        self.system_metrics = {
            'start_time': time.time(),
            'uptime': 0,
            'total_distance': 0,
            'obstacles_detected': 0,
            'decisions_made': 0,
            'mode_changes': 0,
            'errors': 0
        }
        
        # Inicializar sistema
        self.initialize_system()
    
    def signal_handler(self, signum, frame):
        """Manejo de señales para shutdown limpio"""
        print(f"\n🛑 Señal recibida: {signum}")
        self.shutdown()
        sys.exit(0)
    
    def initialize_system(self):
        """Inicializar todos los componentes del robot"""
        try:
            print("🔧 Inicializando componentes del sistema...")
            
            # 1. Mostrar configuración
            print_config()
            
            # 2. Inicializar control de motores
            print("\n📡 Inicializando control de motores...")
            self.motors = MotorController()
            
            # 3. Inicializar comunicación con Arduino
            if self.enable_arduino:
                print("📡 Inicializando comunicación con Arduino...")
                self.arduino = ArduinoComm()
                
                if not self.arduino.is_connected():
                    print("⚠️  Arduino no conectado - funcionando solo con encoders")
                    self.arduino = None
            else:
                print("⚠️  Arduino deshabilitado por configuración")
                self.arduino = None
            
            # 4. Inicializar sistema de sensor fusion
            print("🧠 Inicializando sistema de sensor fusion...")
            self.sensor_fusion = SensorFusion()
            self.sensor_fusion.start()
            
            # 5. Inicializar navegador
            print("🧭 Inicializando sistema de navegación...")
            self.navigator = RobotNavigator(self.motors, self.sensor_fusion)
            
            # 6. Inicializar cámara si está habilitada
            if self.enable_camera:
                print("📷 Inicializando cámara...")
                self.initialize_camera()
            
            # 7. Configurar Arduino si está disponible
            if self.arduino:
                self.arduino.set_frequency(ULTRASONIC_FREQUENCY)
                self.arduino.enable_sensors(True)
            
            self.initialized = True
            print("\n✅ Sistema completamente inicializado")
            
        except Exception as e:
            print(f"❌ Error inicializando sistema: {e}")
            self.shutdown()
            sys.exit(1)
    
    def initialize_camera(self):
        """Inicializar cámara USB"""
        try:
            self.camera = cv2.VideoCapture(CAMERA_DEVICE)
            self.camera.set(cv2.CAP_PROP_FRAME_WIDTH, CAMERA_WIDTH)
            self.camera.set(cv2.CAP_PROP_FRAME_HEIGHT, CAMERA_HEIGHT)
            self.camera.set(cv2.CAP_PROP_FPS, CAMERA_FPS)
            
            if not self.camera.isOpened():
                print("⚠️  No se pudo abrir la cámara")
                self.camera = None
                self.enable_camera = False
            else:
                print(f"✅ Cámara inicializada: {CAMERA_WIDTH}x{CAMERA_HEIGHT}@{CAMERA_FPS}fps")
                
        except Exception as e:
            print(f"❌ Error inicializando cámara: {e}")
            self.camera = None
            self.enable_camera = False
    
    def start_system(self):
        """Iniciar operación del robot"""
        if not self.initialized:
            print("❌ Sistema no inicializado")
            return False
        
        try:
            print("\n🚀 === INICIANDO OPERACIÓN DEL ROBOT ===")
            
            self.running = True
            
            # Iniciar navegador
            self.navigator.start()
            
            # Iniciar threads de monitoreo
            self.start_monitoring_threads()
            
            print("✅ Robot operativo")
            return True
            
        except Exception as e:
            print(f"❌ Error iniciando sistema: {e}")
            self.system_metrics['errors'] += 1
            return False
    
    def start_monitoring_threads(self):
        """Iniciar threads de monitoreo del sistema"""
        # Thread principal de monitoreo
        self.monitor_thread = threading.Thread(target=self._monitoring_loop, daemon=True)
        self.monitor_thread.start()
        
        # Thread de cámara si está habilitada
        if self.enable_camera and self.camera:
            self.camera_thread = threading.Thread(target=self._camera_loop, daemon=True)
            self.camera_thread.start()
    
    def _monitoring_loop(self):
        """Loop principal de monitoreo del sistema"""
        last_odometry_update = time.time()
        last_sensor_update = time.time()
        
        while self.running:
            try:
                current_time = time.time()
                
                # Actualizar métricas del sistema
                self.system_metrics['uptime'] = current_time - self.system_metrics['start_time']
                
                # Actualizar odometría
                if current_time - last_odometry_update >= (1.0 / ODOMETRY_UPDATE_FREQUENCY):
                    self._update_odometry()
                    last_odometry_update = current_time
                
                # Actualizar sensores Arduino
                if self.arduino and (current_time - last_sensor_update >= (1.0 / SENSOR_FUSION_FREQUENCY)):
                    self._update_arduino_sensors()
                    last_sensor_update = current_time
                
                # Verificar estado del sistema
                self._check_system_health()
                
                time.sleep(0.02)  # 50Hz loop
                
            except Exception as e:
                print(f"❌ Error en monitoring loop: {e}")
                self.system_metrics['errors'] += 1
                time.sleep(0.1)
    
    def _update_odometry(self):
        """Actualizar odometría basada en encoders"""
        try:
            # Obtener distancias recorridas desde la última actualización
            left_dist, right_dist = self.motors.get_distance_traveled()
            
            # Calcular diferencial desde última lectura
            if hasattr(self, '_last_left_dist'):
                delta_left = left_dist - self._last_left_dist
                delta_right = right_dist - self._last_right_dist
                dt = time.time() - self._last_odometry_time
                
                # Actualizar sensor fusion
                self.sensor_fusion.update_odometry(delta_left, delta_right, dt)
                
                # Actualizar métricas
                avg_delta = (delta_left + delta_right) / 2.0
                self.system_metrics['total_distance'] += avg_delta
            
            # Guardar valores para próxima iteración
            self._last_left_dist = left_dist
            self._last_right_dist = right_dist
            self._last_odometry_time = time.time()
            
        except Exception as e:
            print(f"❌ Error actualizando odometría: {e}")
            self.system_metrics['errors'] += 1
    
    def _update_arduino_sensors(self):
        """Actualizar sensores desde Arduino"""
        try:
            if self.arduino and self.arduino.is_connected():
                sensor_data = self.arduino.get_latest_data()
                
                if sensor_data and sensor_data.get('timestamp', 0) > 0:
                    self.sensor_fusion.update_sensors(sensor_data)
                    
                    # Contar obstáculos detectados
                    for distance in sensor_data.values():
                        if isinstance(distance, (int, float)) and 0 < distance < OBSTACLE_THRESHOLD_CM:
                            self.system_metrics['obstacles_detected'] += 1
                            break
            
        except Exception as e:
            print(f"❌ Error actualizando sensores Arduino: {e}")
            self.system_metrics['errors'] += 1
    
    def _camera_loop(self):
        """Loop de procesamiento de cámara"""
        while self.running and self.camera:
            try:
                ret, frame = self.camera.read()
                
                if ret:
                    # Aquí se puede agregar procesamiento de visión por computadora
                    # Por ahora solo guardamos el frame más reciente
                    self._latest_camera_frame = frame.copy()
                else:
                    print("⚠️  Error leyendo frame de cámara")
                    time.sleep(0.1)
                
                time.sleep(1.0 / CAMERA_FPS)  # Controlar FPS
                
            except Exception as e:
                print(f"❌ Error en camera loop: {e}")
                self.system_metrics['errors'] += 1
                time.sleep(0.5)
    
    def _check_system_health(self):
        """Verificar salud del sistema"""
        try:
            # Verificar conexión Arduino
            if self.arduino and not self.arduino.is_connected():
                print("⚠️  Conexión Arduino perdida")
            
            # Verificar threads activos
            if not self.navigator.running:
                print("⚠️  Navegador no está corriendo")
            
            # Verificar tiempo de respuesta
            # (implementar si es necesario)
            
        except Exception as e:
            print(f"❌ Error verificando salud del sistema: {e}")
    
    def set_navigation_mode(self, mode_name):
        """Cambiar modo de navegación"""
        try:
            mode_map = {
                'idle': NavigationMode.IDLE,
                'explore': NavigationMode.EXPLORE,
                'avoid': NavigationMode.AVOID_OBSTACLES,
                'goto': NavigationMode.GOTO_POINT,
                'wall': NavigationMode.FOLLOW_WALL,
                'competition': NavigationMode.COMPETITION
            }
            
            if mode_name in mode_map:
                self.navigator.set_mode(mode_map[mode_name])
                self.system_metrics['mode_changes'] += 1
                print(f"🔄 Modo cambiado a: {mode_name}")
                return True
            else:
                print(f"❌ Modo desconocido: {mode_name}")
                return False
                
        except Exception as e:
            print(f"❌ Error cambiando modo: {e}")
            return False
    
    def add_waypoint(self, x, y):
        """Agregar punto de destino"""
        try:
            self.navigator.add_waypoint(x, y)
            print(f"📍 Waypoint agregado: ({x}, {y})")
            return True
        except Exception as e:
            print(f"❌ Error agregando waypoint: {e}")
            return False
    
    def emergency_stop(self):
        """Parada de emergencia"""
        try:
            self.navigator.emergency_stop()
            print("🚨 PARADA DE EMERGENCIA ACTIVADA")
        except Exception as e:
            print(f"❌ Error en parada de emergencia: {e}")
    
    def get_system_status(self):
        """Obtener estado completo del sistema"""
        try:
            status = {
                'system': {
                    'running': self.running,
                    'initialized': self.initialized,
                    'uptime': self.system_metrics['uptime'],
                    'errors': self.system_metrics['errors']
                },
                'robot_state': self.sensor_fusion.get_robot_state() if self.sensor_fusion else {},
                'navigation': self.navigator.get_status() if self.navigator else {},
                'sensors': self.sensor_fusion.get_latest_sensors() if self.sensor_fusion else {},
                'metrics': self.system_metrics.copy()
            }
            
            if self.arduino:
                status['arduino'] = self.arduino.get_statistics()
            
            return status
            
        except Exception as e:
            print(f"❌ Error obteniendo estado: {e}")
            return {'error': str(e)}
    
    def print_status(self):
        """Imprimir estado del sistema en consola"""
        try:
            status = self.get_system_status()
            
            print("\n" + "="*60)
            print("📊 ESTADO DEL ROBOT")
            print("="*60)
            
            # Estado del sistema
            sys_status = status['system']
            print(f"🤖 Sistema: {'🟢 Operativo' if sys_status['running'] else '🔴 Detenido'}")
            print(f"⏱️  Tiempo activo: {sys_status['uptime']:.1f}s")
            print(f"❌ Errores: {sys_status['errors']}")
            
            # Estado del robot
            robot_state = status['robot_state']
            if robot_state:
                print(f"📍 Posición: x={robot_state['x']:.1f}cm, y={robot_state['y']:.1f}cm, θ={math.degrees(robot_state['theta']):.1f}°")
                print(f"🏃 Velocidad: {robot_state['velocity']:.1f}cm/s")
            
            # Estado de navegación
            nav_status = status['navigation']
            if nav_status:
                print(f"🧭 Navegación: {nav_status['mode']} ({nav_status['state']})")
                print(f"🎯 Última acción: {nav_status['last_action']}")
            
            # Sensores
            sensors = status['sensors']
            if sensors:
                print(f"📡 Sensores: F:{sensors.get('front',0)}cm B:{sensors.get('back',0)}cm L:{sensors.get('left',0)}cm R:{sensors.get('right',0)}cm")
            
            # Métricas
            metrics = status['metrics']
            print(f"📊 Distancia total: {metrics['total_distance']:.1f}cm")
            print(f"🚧 Obstáculos detectados: {metrics['obstacles_detected']}")
            print(f"🔄 Cambios de modo: {metrics['mode_changes']}")
            
            # Arduino
            if 'arduino' in status:
                arduino_stats = status['arduino']
                print(f"📟 Arduino: {arduino_stats['success_rate']:.1f}% éxito ({arduino_stats['messages_valid']}/{arduino_stats['messages_received']})")
            
            print("="*60)
            
        except Exception as e:
            print(f"❌ Error imprimiendo estado: {e}")
    
    def interactive_mode(self):
        """Modo interactivo de control"""
        print("\n🎮 === MODO INTERACTIVO ===")
        print("Comandos disponibles:")
        print("  status, s          - Mostrar estado")
        print("  explore            - Modo exploración")
        print("  idle               - Modo reposo")
        print("  goto X Y           - Ir a punto (X,Y)")
        print("  wall               - Seguir pared")
        print("  competition        - Modo competencia")
        print("  emergency, stop    - Parada de emergencia")
        print("  reset              - Reiniciar posición")
        print("  quit, q            - Salir")
        print()
        
        while self.running:
            try:
                cmd = input("🤖 > ").strip().lower().split()
                
                if not cmd:
                    continue
                
                command = cmd[0]
                
                if command in ['quit', 'q', 'exit']:
                    break
                elif command in ['status', 's']:
                    self.print_status()
                elif command == 'explore':
                    self.set_navigation_mode('explore')
                elif command == 'idle':
                    self.set_navigation_mode('idle')
                elif command == 'wall':
                    self.set_navigation_mode('wall')
                elif command == 'competition':
                    self.set_navigation_mode('competition')
                elif command in ['emergency', 'stop']:
                    self.emergency_stop()
                elif command == 'goto' and len(cmd) >= 3:
                    try:
                        x, y = float(cmd[1]), float(cmd[2])
                        self.add_waypoint(x, y)
                        self.set_navigation_mode('goto')
                    except ValueError:
                        print("❌ Coordenadas inválidas")
                elif command == 'reset':
                    self.sensor_fusion.reset_position()
                    self.motors.reset_encoders()
                    print("🔄 Posición reiniciada")
                else:
                    print("❌ Comando no reconocido")
                    
            except (EOFError, KeyboardInterrupt):
                break
            except Exception as e:
                print(f"❌ Error en comando: {e}")
        
        print("👋 Saliendo del modo interactivo...")
    
    def autonomous_demo(self, duration=60):
        """Demo autónomo por tiempo limitado"""
        print(f"\n🤖 === DEMO AUTÓNOMO ({duration}s) ===")
        
        # Iniciar en modo exploración
        self.set_navigation_mode('explore')
        
        start_time = time.time()
        last_status = time.time()
        
        while self.running and (time.time() - start_time) < duration:
            # Mostrar estado cada 10 segundos
            if time.time() - last_status >= 10:
                self.print_status()
                last_status = time.time()
            
            time.sleep(1)
        
        # Detener al final
        self.set_navigation_mode('idle')
        print("✅ Demo autónomo completado")
    
    def competition_mode(self):
        """Modo específico de competencia"""
        print("\n🏆 === MODO COMPETENCIA ===")
        
        # Configurar para competencia
        self.set_navigation_mode('competition')
        
        start_time = time.time()
        
        try:
            while self.running and (time.time() - start_time) < GAME_DURATION:
                time_remaining = GAME_DURATION - (time.time() - start_time)
                
                # Mostrar progreso cada 15 segundos
                if int(time_remaining) % 15 == 0:
                    print(f"⏱️  Tiempo restante: {time_remaining:.0f}s")
                    mission_status = self.navigator.get_mission_status()
                    print(f"🎯 Misión: {mission_status}")
                
                time.sleep(1)
            
            # Tiempo agotado
            self.set_navigation_mode('idle')
            print("🏁 Tiempo de competencia agotado")
            
        except KeyboardInterrupt:
            print("\n🛑 Competencia interrumpida")
            self.set_navigation_mode('idle')
    
    def shutdown(self):
        """Apagar sistema limpiamente"""
        if not self.running:
            return
        
        print("\n🛑 === APAGANDO SISTEMA ===")
        
        self.running = False
        
        try:
            # Detener navegador
            if self.navigator:
                self.navigator.cleanup()
            
            # Detener sensor fusion
            if self.sensor_fusion:
                self.sensor_fusion.cleanup()
            
            # Detener motores
            if self.motors:
                self.motors.cleanup()
            
            # Cerrar Arduino
            if self.arduino:
                self.arduino.cleanup()
            
            # Cerrar cámara
            if self.camera:
                self.camera.release()
                cv2.destroyAllWindows()
            
            print("✅ Sistema apagado correctamente")
            
        except Exception as e:
            print(f"❌ Error durante apagado: {e}")
    
    def __del__(self):
        """Destructor para limpieza automática"""
        self.shutdown()

def main():
    """Función principal"""
    parser = argparse.ArgumentParser(description='Robot Autónomo - Sistema Principal')
    parser.add_argument('--mode', choices=['interactive', 'demo', 'competition'], 
                       default='interactive', help='Modo de operación')
    parser.add_argument('--duration', type=int, default=60, 
                       help='Duración del demo en segundos')
    parser.add_argument('--no-camera', action='store_true', 
                       help='Deshabilitar cámara')
    parser.add_argument('--no-arduino', action='store_true', 
                       help='Deshabilitar Arduino')
    
    args = parser.parse_args()
    
    try:
        # Crear e inicializar sistema
        robot = RobotSystem(
            enable_camera=not args.no_camera,
            enable_arduino=not args.no_arduino
        )
        
        # Iniciar sistema
        if not robot.start_system():
            print("❌ Error iniciando sistema")
            return 1
        
        # Ejecutar modo seleccionado
        if args.mode == 'interactive':
            robot.interactive_mode()
        elif args.mode == 'demo':
            robot.autonomous_demo(args.duration)
        elif args.mode == 'competition':
            robot.competition_mode()
        
        # Apagar sistema
        robot.shutdown()
        
        return 0
        
    except KeyboardInterrupt:
        print("\n🛑 Programa interrumpido por usuario")
        return 0
    except Exception as e:
        print(f"❌ Error fatal: {e}")
        return 1

if __name__ == "__main__":
    import math  # Necesario para print_status
    sys.exit(main())