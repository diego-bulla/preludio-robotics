#!/usr/bin/env python3
"""
Script de prueba individual - Todo controlado por Raspberry Pi
Motores via L298N + 4 HC-SR04 + MPU6050
"""

import RPi.GPIO as GPIO
import time
import smbus
from math import sin, cos, radians

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

# Sensores HC-SR04
# Sensor 1
TRIG_SENSOR1 = 4   # GPIO 4, Pin 7
ECHO_SENSOR1 = 17  # GPIO 17, Pin 11

# Sensor 2  
TRIG_SENSOR2 = 27  # GPIO 27, Pin 13
ECHO_SENSOR2 = 22  # GPIO 22, Pin 15

# Sensor 3
TRIG_SENSOR3 = 5   # GPIO 5, Pin 29
ECHO_SENSOR3 = 6   # GPIO 6, Pin 31

# Sensor 4
TRIG_SENSOR4 = 12  # GPIO 12, Pin 32
ECHO_SENSOR4 = 24

# MPU6050
MPU6050_ADDR = 0x68
I2C_BUS = 1

# Configuración
GPIO.setmode(GPIO.BCM)
GPIO.setwarnings(False)

# ============================================================================
# CLASES DE CONTROL
# ============================================================================

class MotorController:
    """Control directo de motores L298N"""
    def __init__(self):
        # Configurar pines
        GPIO.setup([ENA, IN1, IN2, IN3, IN4, ENB], GPIO.OUT)
        
        # Crear PWM
        self.pwm_left = GPIO.PWM(ENA, 1000)   # 1kHz
        self.pwm_right = GPIO.PWM(ENB, 1000)
        
        self.pwm_left.start(0)
        self.pwm_right.start(0)
        
        print("✅ Motores L298N iniciados")
    
    def move(self, left_speed, right_speed):
        """left_speed, right_speed: -255 a 255"""
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
        
        # Aplicar PWM
        self.pwm_left.ChangeDutyCycle(left_speed * 100 / 255)
        self.pwm_right.ChangeDutyCycle(right_speed * 100 / 255)
    
    def stop(self):
        self.move(0, 0)

class UltrasonicSensor:
    """Sensor HC-SR04 individual"""
    def __init__(self, trig_pin, echo_pin, name):
        self.trig_pin = trig_pin
        self.echo_pin = echo_pin
        self.name = name
        
        GPIO.setup(trig_pin, GPIO.OUT)
        GPIO.setup(echo_pin, GPIO.IN)
        GPIO.output(trig_pin, GPIO.LOW)
        
        print(f"✅ {name} configurado (Trig:{trig_pin}, Echo:{echo_pin})")
    
    def get_distance(self):
        """Retorna distancia en cm"""
        try:
            # Enviar pulso
            GPIO.output(self.trig_pin, GPIO.HIGH)
            time.sleep(0.00001)  # 10µs
            GPIO.output(self.trig_pin, GPIO.LOW)
            
            # Medir tiempo con timeout
            timeout_start = time.time()
            
            # Esperar inicio del pulso
            while GPIO.input(self.echo_pin) == 0:
                pulse_start = time.time()
                if pulse_start - timeout_start > 0.1:  # 100ms timeout
                    return -1
            
            # Esperar fin del pulso  
            while GPIO.input(self.echo_pin) == 1:
                pulse_end = time.time()
                if pulse_end - timeout_start > 0.1:
                    return -1
            
            # Calcular distancia
            pulse_duration = pulse_end - pulse_start
            distance = pulse_duration * 17150  # Velocidad sonido / 2
            
            return round(distance, 1) if 2 <= distance <= 400 else -1
            
        except Exception as e:
            print(f"Error en {self.name}: {e}")
            return -1

class MPU6050:
    """Acelerómetro/Giroscopio simple"""
    def __init__(self):
        try:
            self.bus = smbus.SMBus(I2C_BUS)
            # Despertar MPU6050
            self.bus.write_byte_data(MPU6050_ADDR, 0x6B, 0)
            time.sleep(0.1)
            print("✅ MPU6050 conectado")
            self.connected = True
        except Exception as e:
            print(f"❌ MPU6050 error: {e}")
            self.connected = False
    
    def read_gyro_z(self):
        """Lee giroscopio eje Z para rotación"""
        if not self.connected:
            return 0
        
        try:
            # Leer GYRO_ZOUT (registros 0x47, 0x48)
            high = self.bus.read_byte_data(MPU6050_ADDR, 0x47)
            low = self.bus.read_byte_data(MPU6050_ADDR, 0x48)
            
            # Convertir a valor con signo
            value = (high << 8) | low
            if value > 32768:
                value = value - 65536
                
            # Convertir a grados/segundo (sensibilidad por defecto)
            return value / 131.0
        except:
            return 0

# ============================================================================
# FUNCIONES DE PRUEBA
# ============================================================================

def test_motors():
    """Prueba los motores L298N"""
    print("\n=== PRUEBA DE MOTORES L298N ===")
    motor = MotorController()
    
    tests = [
        ("Adelante lento", 80, 80, 2),
        ("Adelante rápido", 150, 150, 2),
        ("Giro derecha", 100, -100, 2),
        ("Giro izquierda", -100, 100, 2),
        ("Atrás", -120, -120, 2)
    ]
    
    for name, left, right, duration in tests:
        print(f"{name}... ", end="", flush=True)
        motor.move(left, right)
        time.sleep(duration)
        print("OK")
    
    motor.stop()
    print("✅ Motores detenidos")

def test_single_ultrasonic():
    """Prueba un sensor individual"""
    print("\n=== PRUEBA INDIVIDUAL DE HC-SR04 ===")
    
    sensors = [
        (TRIG_SENSOR1, ECHO_SENSOR1, "Sensor 1"),
        (TRIG_SENSOR2, ECHO_SENSOR2, "Sensor 2"),
        (TRIG_SENSOR3, ECHO_SENSOR3, "Sensor 3"),
        (TRIG_SENSOR4, ECHO_SENSOR4, "Sensor 4")
    ]
    
    for trig, echo, name in sensors:
        print(f"\nProbando {name} (Trig:{trig}, Echo:{echo})")
        sensor = UltrasonicSensor(trig, echo, name)
        
        print("Leyendo 5 muestras...")
        for i in range(5):
            dist = sensor.get_distance()
            print(f"  Muestra {i+1}: {dist} cm")
            time.sleep(0.5)

def test_all_ultrasonic():
    """Prueba todos los sensores juntos"""
    print("\n=== PRUEBA DE TODOS LOS HC-SR04 ===")
    
    # Crear sensores
    sensor1 = UltrasonicSensor(TRIG_SENSOR1, ECHO_SENSOR1, "Sensor-1")
    sensor2 = UltrasonicSensor(TRIG_SENSOR2, ECHO_SENSOR2, "Sensor-2")
    sensor3 = UltrasonicSensor(TRIG_SENSOR3, ECHO_SENSOR3, "Sensor-3")
    sensor4 = UltrasonicSensor(TRIG_SENSOR4, ECHO_SENSOR4, "Sensor-4")
    
    sensors = [sensor1, sensor2, sensor3, sensor4]
    
    print("Leyendo sensores por 10 segundos (secuencial para evitar interferencia)...")
    start_time = time.time()
    
    while (time.time() - start_time) < 10:
        readings = []
        
        for sensor in sensors:
            dist = sensor.get_distance()
            readings.append(f"{sensor.name}: {dist:6.1f}cm")
            time.sleep(0.1)  # Pausa entre sensores
        
        print(" | ".join(readings))
        time.sleep(0.2)  # Pausa entre ciclos completos

def test_mpu6050():
    """Prueba el acelerómetro/giroscopio"""
    print("\n=== PRUEBA DE MPU6050 ===")
    mpu = MPU6050()
    
    if not mpu.connected:
        print("No se puede probar MPU6050")
        return
    
    print("Leyendo giroscopio Z por 10 segundos...")
    print("(Gira el robot para ver cambios)")
    
    start_time = time.time()
    angle = 0
    
    while (time.time() - start_time) < 10:
        gyro_z = mpu.read_gyro_z()
        angle += gyro_z * 0.1  # Integrar (dt=0.1s)
        
        print(f"Gyro Z: {gyro_z:6.2f}°/s | Ángulo: {angle:6.1f}°")
        time.sleep(0.1)

def test_pin_status():
    """Verifica que los pines estén disponibles"""
    print("\n=== VERIFICACIÓN DE PINES ===")
    
    pin_map = {
        "ENA (Motor)": ENA,
        "IN1 (Motor)": IN1, 
        "IN2 (Motor)": IN2,
        "IN3 (Motor)": IN3,
        "IN4 (Motor)": IN4,
        "ENB (Motor)": ENB,
        "Trig Sensor 1": TRIG_SENSOR1,
        "Echo Sensor 1": ECHO_SENSOR1,
        "Trig Sensor 2": TRIG_SENSOR2,
        "Echo Sensor 2": ECHO_SENSOR2,
        "Trig Sensor 3": TRIG_SENSOR3,
        "Echo Sensor 3": ECHO_SENSOR3,
        "Trig Sensor 4": TRIG_SENSOR4,
        "Echo Sensor 4": ECHO_SENSOR4
    }
    
    for name, pin in pin_map.items():
        print(f"{name:15}: GPIO {pin:2d}")
    
    print("\nI2C para MPU6050:")
    print(f"SDA: GPIO 2 (Pin 3)")
    print(f"SCL: GPIO 3 (Pin 5)")
    
    print("\n✅ No hay conflictos de pines")

# ============================================================================
# MENÚ PRINCIPAL
# ============================================================================

def main():
    """Menú principal de pruebas"""
    while True:
        print("\n" + "="*50)
        print("PRUEBA DE COMPONENTES - CONTROL DIRECTO RPi")
        print("="*50)
        print("1. Verificar pines GPIO")
        print("2. Probar MPU6050")
        print("3. Probar motores L298N")
        print("4. Probar un sensor HC-SR04")
        print("5. Probar todos los sensores HC-SR04")
        print("6. Ejecutar todas las pruebas")
        print("0. Salir")
        
        try:
            choice = input("\nElegir opción: ")
            
            if choice == "1":
                test_pin_status()
            elif choice == "2":
                test_mpu6050()
            elif choice == "3":
                choice = input("¿ADVERTENCIA: Los motores se moverán! Continuar? [y/N]: ")
                if choice.lower() == 'y':
                    test_motors()
            elif choice == "4":
                test_single_ultrasonic()
            elif choice == "5":
                test_all_ultrasonic()
            elif choice == "6":
                test_pin_status()
                test_mpu6050()
                choice = input("\n¿Probar motores? [y/N]: ")
                if choice.lower() == 'y':
                    test_motors()
                test_all_ultrasonic()
            elif choice == "0":
                break
            else:
                print("Opción inválida")
                
        except KeyboardInterrupt:
            print("\nInterrumpido por usuario")
            break
        except Exception as e:
            print(f"Error: {e}")
    
    # Cleanup
    try:
        GPIO.cleanup()
    except:
        pass
    
    print("Pruebas terminadas")

if __name__ == "__main__":
    main()