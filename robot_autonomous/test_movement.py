#!/usr/bin/env python3
# test_movement.py - Script Completo de Pruebas de Movimiento con Encoders

import time
import signal
import sys
import threading
import math
from motor_control import MotorController
from arduino_comm import ArduinoComm
from config import *

class MovementTester:
    def __init__(self):
        print("🚀 Iniciando Test de Movimiento del Robot")
        print("=" * 50)
        
        # Inicializar componentes
        self.motors = None
        self.arduino = None
        self.running = True
        
        # Configurar manejo de señales para salida limpia
        signal.signal(signal.SIGINT, self.signal_handler)
        signal.signal(signal.SIGTERM, self.signal_handler)
        
        # Inicializar hardware
        self.init_hardware()
        
        # Variables de estado
        self.test_results = {
            'basic_movements': {},
            'speed_tests': {},
            'precision_tests': {},
            'sensor_integration': {}
        }
    
    def init_hardware(self):
        """Inicializar hardware del robot"""
        try:
            print("🔧 Inicializando hardware...")
            
            # Inicializar control de motores
            print("  📡 Conectando motores...")
            self.motors = MotorController()
            
            # Inicializar comunicación con Arduino
            print("  📡 Conectando Arduino...")
            self.arduino = ArduinoComm()
            
            if self.arduino.is_connected():
                print("  ✅ Arduino conectado")
                # Configurar frecuencia de sensores
                self.arduino.set_frequency(ULTRASONIC_FREQUENCY)
            else:
                print("  ⚠️  Arduino no conectado - solo pruebas de motores")
                self.arduino = None
            
            print("✅ Hardware inicializado correctamente\n")
            
        except Exception as e:
            print(f"❌ Error inicializando hardware: {e}")
            self.cleanup()
            sys.exit(1)
    
    def signal_handler(self, signum, frame):
        """Manejo de señal para salida limpia"""
        print("\n🛑 Deteniendo pruebas...")
        self.running = False
        self.cleanup()
        sys.exit(0)
    
    def print_header(self, title):
        """Imprimir encabezado de sección"""
        print("\n" + "=" * 60)
        print(f"🧪 {title}")
        print("=" * 60)
    
    def print_sensor_data(self):
        """Imprimir datos actuales de sensores"""
        sensor_data = ""
        
        # Datos de encoders
        left_count, right_count = self.motors.get_encoder_counts()
        left_dist, right_dist = self.motors.get_distance_traveled()
        avg_dist = self.motors.get_average_distance()
        
        sensor_data += f"📊 Encoders - L:{left_count:4d}({left_dist:6.2f}cm) R:{right_count:4d}({right_dist:6.2f}cm) Avg:{avg_dist:6.2f}cm"
        
        # Datos de ultrasonidos si disponibles
        if self.arduino:
            ultrasonic_data = self.arduino.get_latest_data()
            sensor_data += f" | 📡 Ultra - F:{ultrasonic_data['front']:3d} B:{ultrasonic_data['back']:3d} L:{ultrasonic_data['left']:3d} R:{ultrasonic_data['right']:3d}cm"
        
        print(sensor_data)
    
    def wait_with_monitoring(self, duration, message=""):
        """Esperar tiempo específico mientras monitorea sensores"""
        if message:
            print(f"⏱️  {message}")
        
        start_time = time.time()
        last_print = time.time()
        
        while (time.time() - start_time) < duration and self.running:
            # Imprimir datos cada PRINT_INTERVAL
            if time.time() - last_print >= PRINT_INTERVAL:
                self.print_sensor_data()
                last_print = time.time()
            
            time.sleep(0.05)  # Pequeña pausa para no saturar CPU
    
    def test_basic_movements(self):
        """Prueba movimientos básicos con monitoreo completo"""
        self.print_header("MOVIMIENTOS BÁSICOS")
        
        movements = [
            ("Adelante", self.motors.move_forward, SPEED_NORMAL),
            ("Atrás", self.motors.move_backward, SPEED_NORMAL),
            ("Giro Izquierda", self.motors.turn_left, SPEED_TURN),
            ("Giro Derecha", self.motors.turn_right, SPEED_TURN),
            ("Spin Izquierda", self.motors.spin_left, SPEED_TURN),
            ("Spin Derecha", self.motors.spin_right, SPEED_TURN)
        ]
        
        for name, movement_func, speed in movements:
            if not self.running:
                break
            
            print(f"\n🎯 Probando: {name}")
            print("-" * 40)
            
            # Reiniciar encoders
            self.motors.reset_encoders()
            
            # Ejecutar movimiento
            movement_func(speed)
            
            # Monitorear durante el movimiento
            self.wait_with_monitoring(TEST_DURATION, f"Ejecutando {name}...")
            
            # Detener motores
            self.motors.stop_motors()
            
            # Recopilar resultados
            left_dist, right_dist = self.motors.get_distance_traveled()
            avg_dist = self.motors.get_average_distance()
            
            result = {
                'left_distance': left_dist,
                'right_distance': right_dist,
                'average_distance': avg_dist,
                'duration': TEST_DURATION,
                'speed_used': speed
            }
            
            self.test_results['basic_movements'][name] = result
            
            print(f"\n📋 RESUMEN {name}:")
            print(f"   Distancia promedio: {avg_dist:.2f}cm")
            print(f"   Diferencia L-R: {abs(left_dist - right_dist):.2f}cm")
            print(f"   Velocidad efectiva: {avg_dist/TEST_DURATION:.2f}cm/s")
            
            # Pausa entre movimientos
            print("⏸️  Pausa...")
            time.sleep(1)
    
    def test_speed_variations(self):
        """Prueba diferentes velocidades y mide rendimiento"""
        self.print_header("PRUEBA DE VELOCIDADES")
        
        speeds = [30, 50, 70, 90]
        test_duration = 3.0
        
        for speed in speeds:
            if not self.running:
                break
            
            print(f"\n🎯 Probando velocidad: {speed}%")
            print("-" * 30)
            
            self.motors.reset_encoders()
            self.motors.move_forward(speed)
            
            self.wait_with_monitoring(test_duration, f"Velocidad {speed}% por {test_duration}s")
            
            self.motors.stop_motors()
            
            # Calcular métricas
            left_dist, right_dist = self.motors.get_distance_traveled()
            avg_dist = self.motors.get_average_distance()
            avg_speed_cm_s = avg_dist / test_duration
            
            result = {
                'commanded_speed': speed,
                'distance_traveled': avg_dist,
                'actual_speed_cm_s': avg_speed_cm_s,
                'duration': test_duration,
                'efficiency': (avg_speed_cm_s / speed) * 100 if speed > 0 else 0
            }
            
            self.test_results['speed_tests'][f'speed_{speed}'] = result
            
            print(f"📈 Velocidad comandada: {speed}%")
            print(f"📈 Velocidad real: {avg_speed_cm_s:.2f} cm/s")
            print(f"📈 Eficiencia: {result['efficiency']:.1f}%")
            
            time.sleep(1)
    
    def test_precise_distance(self, target_distance_cm=TEST_DISTANCE_CM):
        """Prueba movimiento a distancia específica con alta precisión"""
        self.print_header(f"MOVIMIENTO PRECISO - {target_distance_cm}cm")
        
        self.motors.reset_encoders()
        self.motors.move_forward(SPEED_SLOW)  # Velocidad lenta para mayor precisión
        
        print(f"🎯 Objetivo: {target_distance_cm}cm")
        print("📊 Progreso:")
        
        start_time = time.time()
        last_distance = 0
        
        while self.running:
            current_distance = self.motors.get_average_distance()
            progress = (current_distance / target_distance_cm) * 100
            
            # Barra de progreso visual
            bar_length = 30
            filled_length = int(bar_length * progress / 100)
            bar = "█" * filled_length + "░" * (bar_length - filled_length)
            
            print(f"   {current_distance:6.2f}cm ({progress:5.1f}%) |{bar}|")
            
            # Verificar si alcanzamos el objetivo
            if current_distance >= target_distance_cm:
                break
            
            # Control de velocidad adaptativo
            remaining = target_distance_cm - current_distance
            if remaining < 5:  # Últimos 5cm más despacio
                self.motors.move_forward(SPEED_SLOW // 2)
            
            last_distance = current_distance
            time.sleep(PRINT_INTERVAL)
        
        self.motors.stop_motors()
        
        # Análisis de precisión
        final_distance = self.motors.get_average_distance()
        error = abs(final_distance - target_distance_cm)
        error_percentage = (error / target_distance_cm) * 100
        time_taken = time.time() - start_time
        
        result = {
            'target_distance': target_distance_cm,
            'actual_distance': final_distance,
            'error_cm': error,
            'error_percentage': error_percentage,
            'time_taken': time_taken,
            'precision_rating': 'Excelente' if error < 1 else 'Buena' if error < 3 else 'Aceptable' if error < 5 else 'Pobre'
        }
        
        self.test_results['precision_tests']['distance_test'] = result
        
        print(f"\n✅ Completado!")
        print(f"📏 Distancia objetivo: {target_distance_cm:.2f}cm")
        print(f"📏 Distancia final: {final_distance:.2f}cm")
        print(f"🎯 Error: {error:.2f}cm ({error_percentage:.1f}%)")
        print(f"⏱️  Tiempo: {time_taken:.2f}s")
        print(f"⭐ Precisión: {result['precision_rating']}")
    
    def test_rotation_precision(self, target_degrees=90):
        """Prueba precisión de rotación usando odometría"""
        self.print_header(f"ROTACIÓN PRECISA - {target_degrees}°")
        
        self.motors.reset_encoders()
        
        # Calcular distancia de arco necesaria para la rotación
        arc_length_cm = (target_degrees / 360.0) * (WHEELBASE_CM * math.pi)
        
        print(f"🎯 Rotación objetivo: {target_degrees}°")
        print(f"📐 Longitud de arco calculada: {arc_length_cm:.2f}cm")
        
        self.motors.spin_right(SPEED_TURN)
        
        start_time = time.time()
        
        while self.running:
            left_dist, right_dist = self.motors.get_distance_traveled()
            
            # Calcular rotación actual basada en diferencia de distancias
            distance_diff = abs(right_dist - left_dist)
            current_rotation = (distance_diff / (WHEELBASE_CM * math.pi)) * 360.0
            
            progress = (current_rotation / target_degrees) * 100
            
            print(f"📊 Rotación actual: {current_rotation:.1f}° ({progress:.1f}%)")
            
            if current_rotation >= target_degrees:
                break
            
            time.sleep(PRINT_INTERVAL)
        
        self.motors.stop_motors()
        
        # Análisis de rotación
        final_left_dist, final_right_dist = self.motors.get_distance_traveled()
        final_distance_diff = abs(final_right_dist - final_left_dist)
        final_rotation = (final_distance_diff / (WHEELBASE_CM * math.pi)) * 360.0
        
        rotation_error = abs(final_rotation - target_degrees)
        time_taken = time.time() - start_time
        
        result = {
            'target_rotation': target_degrees,
            'actual_rotation': final_rotation,
            'error_degrees': rotation_error,
            'time_taken': time_taken
        }
        
        self.test_results['precision_tests']['rotation_test'] = result
        
        print(f"\n✅ Rotación completada!")
        print(f"🎯 Objetivo: {target_degrees:.1f}°")
        print(f"📐 Real: {final_rotation:.1f}°")
        print(f"❌ Error: {rotation_error:.1f}°")
        print(f"⏱️  Tiempo: {time_taken:.2f}s")
    
    def test_sensor_integration(self):
        """Prueba integración con sensores ultrasónicos"""
        if not self.arduino:
            print("⚠️  Sensores ultrasónicos no disponibles - saltando prueba")
            return
        
        self.print_header("INTEGRACIÓN CON SENSORES")
        
        print("🧪 Prueba de navegación básica con sensores...")
        print("El robot avanzará hasta detectar un obstáculo")
        
        self.motors.reset_encoders()
        self.motors.move_forward(SPEED_SLOW)
        
        start_time = time.time()
        obstacle_detected = False
        
        while self.running and (time.time() - start_time) < 30:  # Máximo 30 segundos
            # Obtener datos de sensores
            sensor_data = self.arduino.get_latest_data()
            front_distance = sensor_data['front']
            
            # Mostrar información
            self.print_sensor_data()
            
            # Verificar obstáculo al frente
            if front_distance > 0 and front_distance < OBSTACLE_THRESHOLD_CM:
                print(f"🚨 Obstáculo detectado a {front_distance}cm!")
                obstacle_detected = True
                break
            
            time.sleep(PRINT_INTERVAL)
        
        self.motors.stop_motors()
        
        # Recopilar resultados
        distance_traveled = self.motors.get_average_distance()
        final_sensor_data = self.arduino.get_latest_data()
        
        result = {
            'obstacle_detected': obstacle_detected,
            'distance_traveled': distance_traveled,
            'final_sensor_readings': final_sensor_data,
            'detection_threshold': OBSTACLE_THRESHOLD_CM
        }
        
        self.test_results['sensor_integration']['obstacle_detection'] = result
        
        print(f"\n📋 Resultado de integración:")
        print(f"   Obstáculo detectado: {'Sí' if obstacle_detected else 'No'}")
        print(f"   Distancia recorrida: {distance_traveled:.2f}cm")
        if obstacle_detected:
            print(f"   Distancia al obstáculo: {final_sensor_data['front']}cm")
    
    def interactive_control(self):
        """Control interactivo con teclado y monitoreo en tiempo real"""
        self.print_header("CONTROL INTERACTIVO")
        
        print("🎮 Controles disponibles:")
        print("  w - Adelante          s - Atrás")
        print("  a - Izquierda (curva) d - Derecha (curva)")
        print("  z - Giro izq (sitio)  c - Giro der (sitio)")
        print("  x - Parar             r - Reset encoders")
        print("  + - Aumentar vel      - - Disminuir vel")
        print("  t - Test sensores     q - Salir")
        print("\nPresiona Enter después de cada comando...")
        
        current_speed = DEFAULT_SPEED
        
        while self.running:
            try:
                # Mostrar estado actual
                print(f"\n🎮 Velocidad actual: {current_speed}%")
                self.print_sensor_data()
                
                cmd = input("Comando: ").lower().strip()
                
                if cmd == 'w':
                    self.motors.move_forward(current_speed)
                elif cmd == 's':
                    self.motors.move_backward(current_speed)
                elif cmd == 'a':
                    self.motors.turn_left(current_speed)
                elif cmd == 'd':
                    self.motors.turn_right(current_speed)
                elif cmd == 'z':
                    self.motors.spin_left(current_speed)
                elif cmd == 'c':
                    self.motors.spin_right(current_speed)
                elif cmd == 'x':
                    self.motors.stop_motors()
                elif cmd == 'r':
                    self.motors.reset_encoders()
                elif cmd == '+':
                    current_speed = min(current_speed + 10, MAX_SPEED)
                    print(f"➕ Velocidad: {current_speed}%")
                elif cmd == '-':
                    current_speed = max(current_speed - 10, MIN_SPEED)
                    print(f"➖ Velocidad: {current_speed}%")
                elif cmd == 't':
                    if self.arduino:
                        self.arduino.test_sensors()
                        print("🧪 Test de sensores enviado")
                    else:
                        print("❌ Arduino no disponible")
                elif cmd == 'q':
                    break
                else:
                    print("❌ Comando no reconocido")
                    
            except (EOFError, KeyboardInterrupt):
                break
        
        self.motors.stop_motors()
    
    def generate_report(self):
        """Generar reporte completo de las pruebas"""
        self.print_header("REPORTE DE PRUEBAS")
        
        print("📊 RESUMEN DE RESULTADOS:\n")
        
        # Reporte de movimientos básicos
        if self.test_results['basic_movements']:
            print("🚗 MOVIMIENTOS BÁSICOS:")
            for movement, result in self.test_results['basic_movements'].items():
                print(f"   {movement}: {result['average_distance']:.2f}cm en {result['duration']}s")
        
        # Reporte de velocidades
        if self.test_results['speed_tests']:
            print("\n⚡ PRUEBAS DE VELOCIDAD:")
            for test, result in self.test_results['speed_tests'].items():
                print(f"   {result['commanded_speed']}%: {result['actual_speed_cm_s']:.2f}cm/s (Ef: {result['efficiency']:.1f}%)")
        
        # Reporte de precisión
        if self.test_results['precision_tests']:
            print("\n🎯 PRECISIÓN:")
            if 'distance_test' in self.test_results['precision_tests']:
                dt = self.test_results['precision_tests']['distance_test']
                print(f"   Distancia: Error {dt['error_cm']:.2f}cm ({dt['error_percentage']:.1f}%) - {dt['precision_rating']}")
            
            if 'rotation_test' in self.test_results['precision_tests']:
                rt = self.test_results['precision_tests']['rotation_test']
                print(f"   Rotación: Error {rt['error_degrees']:.1f}°")
        
        # Reporte de sensores
        if self.test_results['sensor_integration']:
            print("\n📡 INTEGRACIÓN DE SENSORES:")
            if 'obstacle_detection' in self.test_results['sensor_integration']:
                si = self.test_results['sensor_integration']['obstacle_detection']
                print(f"   Detección de obstáculos: {'✅ Funcionando' if si['obstacle_detected'] else '❌ No detectado'}")
        
        # Estadísticas de comunicación
        if self.arduino:
            stats = self.arduino.get_statistics()
            print(f"\n📊 COMUNICACIÓN ARDUINO:")
            print(f"   Mensajes válidos: {stats['messages_valid']}/{stats['messages_received']}")
            print(f"   Tasa de éxito: {stats['success_rate']:.1f}%")
            print(f"   Errores de checksum: {stats['checksum_errors']}")
    
    def run_all_tests(self):
        """Ejecutar todas las pruebas automáticas"""
        try:
            print("🤖 === INICIANDO SUITE COMPLETA DE PRUEBAS ===")
            print_config()  # Mostrar configuración actual
            
            # Ejecutar pruebas secuencialmente
            tests = [
                ("Movimientos Básicos", self.test_basic_movements),
                ("Velocidades", self.test_speed_variations),
                ("Precisión de Distancia", lambda: self.test_precise_distance(TEST_DISTANCE_CM)),
                ("Precisión de Rotación", lambda: self.test_rotation_precision(TEST_ROTATION_DEGREES)),
                ("Integración de Sensores", self.test_sensor_integration)
            ]
            
            for test_name, test_func in tests:
                if not self.running:
                    break
                
                print(f"\n🔄 Iniciando: {test_name}")
                test_func()
                
                if self.running:
                    print(f"✅ {test_name} completado")
                    time.sleep(2)  # Pausa entre pruebas
            
            # Generar reporte final
            if self.running:
                self.generate_report()
                
                # Preguntar si quiere control interactivo
                try:
                    response = input("\n¿Deseas activar el control interactivo? (y/n): ").lower()
                    if response == 'y':
                        self.interactive_control()
                except:
                    pass
                
        except Exception as e:
            print(f"❌ Error durante las pruebas: {e}")
        finally:
            self.cleanup()
            print("\n✅ Suite de pruebas completada")
    
    def cleanup(self):
        """Limpiar recursos"""
        print("\n🧹 Limpiando recursos...")
        
        if self.motors:
            self.motors.cleanup()
        
        if self.arduino:
            self.arduino.cleanup()
        
        print("✅ Limpieza completada")

def main():
    """Función principal"""
    print("🚀 Test de Movimiento del Robot Autónomo")
    print(f"Version: {VERSION}")
    print("-" * 50)
    
    tester = MovementTester()
    tester.run_all_tests()

if __name__ == "__main__":
    main()