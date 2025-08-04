#!/usr/bin/env python3
# arduino_comm.py - Comunicación Serial con Arduino

import serial
import threading
import time
import queue
from config import *

class ArduinoComm:
    def __init__(self, port=ARDUINO_PORT, baudrate=ARDUINO_BAUDRATE):
        self.port = port
        self.baudrate = baudrate
        self.serial_conn = None
        self.running = False
        
        # Thread para lectura continua
        self.read_thread = None
        
        # Cola para datos recibidos
        self.data_queue = queue.Queue(maxsize=100)
        
        # Último conjunto de datos válidos
        self.last_sensor_data = {
            'front': 0,
            'back': 0,
            'left': 0,
            'right': 0,
            'timestamp': 0,
            'arduino_time': 0,
            'received_time': 0
        }
        
        # Lock para thread safety
        self.data_lock = threading.Lock()
        
        # Estadísticas de comunicación
        self.stats = {
            'messages_received': 0,
            'messages_valid': 0,
            'checksum_errors': 0,
            'parse_errors': 0,
            'connection_errors': 0
        }
        
        # Intentar conectar
        self.connect()
    
    def connect(self):
        """Establecer conexión serial con Arduino"""
        try:
            self.serial_conn = serial.Serial(
                self.port, 
                self.baudrate, 
                timeout=SERIAL_TIMEOUT,
                write_timeout=SERIAL_WRITE_TIMEOUT
            )
            
            # Esperar a que Arduino se inicialice
            time.sleep(2)
            
            # Limpiar buffer
            self.serial_conn.flushInput()
            self.serial_conn.flushOutput()
            
            print(f"✅ Conectado a Arduino en {self.port}")
            
            # Verificar conexión
            if self.test_connection():
                self.running = True
                self.start_reading()
                return True
            else:
                print("❌ Error: Arduino no responde correctamente")
                return False
                
        except serial.SerialException as e:
            print(f"❌ Error de conexión serial: {e}")
            self.stats['connection_errors'] += 1
            return False
        except Exception as e:
            print(f"❌ Error inesperado: {e}")
            return False
    
    def test_connection(self):
        """Probar conexión enviando comando STATUS"""
        try:
            self.send_command("STATUS")
            time.sleep(0.5)
            
            # Leer respuesta
            if self.serial_conn.in_waiting > 0:
                response = self.serial_conn.readline().decode('utf-8').strip()
                if response.startswith("STATUS|"):
                    print(f"📡 Arduino responde: {response}")
                    return True
            
            return False
            
        except Exception as e:
            print(f"❌ Error en test de conexión: {e}")
            return False
    
    def start_reading(self):
        """Iniciar thread de lectura continua"""
        if self.read_thread is None or not self.read_thread.is_alive():
            self.running = True
            self.read_thread = threading.Thread(target=self._read_loop, daemon=True)
            self.read_thread.start()
            print("✅ Thread de lectura iniciado")
    
    def stop_reading(self):
        """Detener thread de lectura"""
        self.running = False
        if self.read_thread and self.read_thread.is_alive():
            self.read_thread.join(timeout=2)
        print("🛑 Thread de lectura detenido")
    
    def _read_loop(self):
        """Loop principal de lectura de datos"""
        consecutive_errors = 0
        
        while self.running:
            try:
                if self.serial_conn and self.serial_conn.in_waiting > 0:
                    line = self.serial_conn.readline().decode('utf-8').strip()
                    
                    if line:
                        self.stats['messages_received'] += 1
                        
                        # Procesar línea recibida
                        if self._process_message(line):
                            consecutive_errors = 0
                        else:
                            consecutive_errors += 1
                    
                else:
                    time.sleep(0.01)  # Pequeña pausa si no hay datos
                
                # Si hay muchos errores consecutivos, intentar reconectar
                if consecutive_errors > MAX_CONSECUTIVE_ERRORS:
                    print(f"⚠️  Demasiados errores consecutivos ({consecutive_errors}), intentando reconectar...")
                    self.reconnect()
                    consecutive_errors = 0
                    
            except serial.SerialException as e:
                print(f"❌ Error de lectura serial: {e}")
                self.stats['connection_errors'] += 1
                time.sleep(1)
                
            except Exception as e:
                print(f"❌ Error inesperado en lectura: {e}")
                consecutive_errors += 1
                time.sleep(0.1)
    
    def _process_message(self, message):
        """Procesar mensaje recibido del Arduino"""
        try:
            if message.startswith("START|"):
                return self._parse_sensor_data(message)
            elif message.startswith("ERR|"):
                self._handle_error_message(message)
                return True
            elif message.startswith("ACK|"):
                print(f"📥 ACK: {message}")
                return True
            elif message.startswith("STATUS|"):
                print(f"📊 Status: {message}")
                return True
            else:
                print(f"⚠️  Mensaje no reconocido: {message}")
                self.stats['parse_errors'] += 1
                return False
                
        except Exception as e:
            print(f"❌ Error procesando mensaje: {e}")
            self.stats['parse_errors'] += 1
            return False
    
    def _parse_sensor_data(self, message):
        """Parsear datos de sensores"""
        try:
            # Formato esperado: START|F:XXX|B:XXX|L:XXX|R:XXX|T:XXXXXX|CHK:XX
            parts = message.split('|')
            
            if len(parts) < 7:
                self.stats['parse_errors'] += 1
                return False
            
            # Extraer valores
            front = int(parts[1].split(':')[1])
            back = int(parts[2].split(':')[1])
            left = int(parts[3].split(':')[1])
            right = int(parts[4].split(':')[1])
            arduino_time = int(parts[5].split(':')[1])
            received_checksum = int(parts[6].split(':')[1])
            
            # Verificar checksum
            calculated_checksum = (front + back + left + right) % 100
            
            if calculated_checksum != received_checksum:
                self.stats['checksum_errors'] += 1
                print(f"❌ Error de checksum: calc={calculated_checksum}, recv={received_checksum}")
                return False
            
            # Datos válidos - actualizar
            current_time = time.time()
            
            with self.data_lock:
                self.last_sensor_data = {
                    'front': front if front > 0 else self.last_sensor_data['front'],
                    'back': back if back > 0 else self.last_sensor_data['back'],
                    'left': left if left > 0 else self.last_sensor_data['left'],
                    'right': right if right > 0 else self.last_sensor_data['right'],
                    'timestamp': current_time,
                    'arduino_time': arduino_time,
                    'received_time': current_time
                }
            
            # Agregar a cola (no bloqueante)
            try:
                self.data_queue.put_nowait(self.last_sensor_data.copy())
            except queue.Full:
                # Si la cola está llena, remover el más antiguo
                try:
                    self.data_queue.get_nowait()
                    self.data_queue.put_nowait(self.last_sensor_data.copy())
                except queue.Empty:
                    pass
            
            self.stats['messages_valid'] += 1
            return True
            
        except (ValueError, IndexError) as e:
            print(f"❌ Error parseando datos de sensores: {e}")
            self.stats['parse_errors'] += 1
            return False
    
    def _handle_error_message(self, message):
        """Manejar mensajes de error del Arduino"""
        print(f"⚠️  Error desde Arduino: {message}")
        
        # Registrar tipos específicos de error
        if "SENSOR:" in message:
            print("🔧 Error de sensor detectado")
        elif "TIMEOUT" in message:
            print("⏱️  Timeout de sensor")
        elif "CHECKSUM" in message:
            print("📊 Error de checksum en Arduino")
    
    def get_latest_data(self):
        """Obtener los últimos datos de sensores"""
        with self.data_lock:
            return self.last_sensor_data.copy()
    
    def get_data_queue(self):
        """Obtener todos los datos en cola (vacía la cola)"""
        data_list = []
        while not self.data_queue.empty():
            try:
                data_list.append(self.data_queue.get_nowait())
            except queue.Empty:
                break
        return data_list
    
    def send_command(self, command):
        """Enviar comando al Arduino"""
        try:
            if self.serial_conn and self.serial_conn.is_open:
                command_line = f"{command}\n"
                self.serial_conn.write(command_line.encode('utf-8'))
                self.serial_conn.flush()
                return True
            else:
                print("❌ Conexión serial no disponible")
                return False
                
        except Exception as e:
            print(f"❌ Error enviando comando: {e}")
            return False
    
    def set_frequency(self, frequency_hz):
        """Configurar frecuencia de lectura del Arduino"""
        if 1 <= frequency_hz <= 20:
            return self.send_command(f"CMD|FREQ:{frequency_hz}|ENABLE:1")
        else:
            print("❌ Frecuencia debe estar entre 1-20 Hz")
            return False
    
    def enable_sensors(self, enable=True):
        """Habilitar/deshabilitar sensores"""
        enable_value = "1" if enable else "0"
        return self.send_command(f"CMD|ENABLE:{enable_value}")
    
    def request_status(self):
        """Solicitar estado del Arduino"""
        return self.send_command("STATUS")
    
    def test_sensors(self):
        """Solicitar una lectura de prueba"""
        return self.send_command("TEST")
    
    def reconnect(self):
        """Intentar reconectar"""
        try:
            if self.serial_conn:
                self.serial_conn.close()
            
            time.sleep(1)
            return self.connect()
            
        except Exception as e:
            print(f"❌ Error en reconexión: {e}")
            return False
    
    def get_statistics(self):
        """Obtener estadísticas de comunicación"""
        stats = self.stats.copy()
        
        if stats['messages_received'] > 0:
            stats['success_rate'] = (stats['messages_valid'] / stats['messages_received']) * 100
        else:
            stats['success_rate'] = 0
            
        return stats
    
    def is_connected(self):
        """Verificar si la conexión está activa"""
        return (self.serial_conn is not None and 
                self.serial_conn.is_open and 
                self.running)
    
    def cleanup(self):
        """Limpiar recursos y cerrar conexión"""
        self.stop_reading()
        
        if self.serial_conn and self.serial_conn.is_open:
            self.serial_conn.close()
        
        print("🧹 ArduinoComm limpiado")

if __name__ == "__main__":
    # Prueba básica de comunicación
    try:
        print("🧪 Iniciando prueba de comunicación con Arduino...")
        
        arduino = ArduinoComm()
        
        if arduino.is_connected():
            print("✅ Conexión establecida")
            
            # Configurar frecuencia
            arduino.set_frequency(10)
            time.sleep(1)
            
            # Leer datos por 10 segundos
            start_time = time.time()
            while time.time() - start_time < 10:
                data = arduino.get_latest_data()
                print(f"📊 F:{data['front']:3d} B:{data['back']:3d} L:{data['left']:3d} R:{data['right']:3d}")
                time.sleep(0.5)
            
            # Mostrar estadísticas
            stats = arduino.get_statistics()
            print(f"\n📈 Estadísticas:")
            print(f"   Mensajes recibidos: {stats['messages_received']}")
            print(f"   Mensajes válidos: {stats['messages_valid']}")
            print(f"   Tasa de éxito: {stats['success_rate']:.1f}%")
            print(f"   Errores de checksum: {stats['checksum_errors']}")
        
        arduino.cleanup()
        
    except KeyboardInterrupt:
        print("\n❌ Prueba interrumpida")
        arduino.cleanup()
    except Exception as e:
        print(f"❌ Error: {e}")
        arduino.cleanup()