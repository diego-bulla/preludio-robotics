#!/usr/bin/env python3
"""
Control de Motor con Encoder usando Raspberry Pi + L298N + Batería 9V
Compatible con encoder magnético Pololu para micro metal gearmotors
"""

import RPi.GPIO as GPIO
import time
import threading
from collections import deque

# Configuración de pines
# Encoder
ENCODER_A = 18  # GPIO 18 (Pin físico 12)
ENCODER_B = 11  # GPIO 19 (Pin físico 23)

# L298N Motor Driver
MOTOR_ENA = 13  # PWM para velocidad (Pin físico 33)
MOTOR_IN1 = 19  # Dirección 1 (Pin físico 35)
MOTOR_IN2 = 16  # Dirección 2 (Pin físico 36)

# Variables globales del encoder
encoder_count = 0
encoder_lock = threading.Lock()
last_a_state = 0
last_b_state = 0

# Variables de control
motor_speed = 50  # Velocidad inicial (0-100%)
target_position = 0
position_control = False

class MotorController:
    def __init__(self):
        self.setup_gpio()
        self.setup_encoder_interrupts()
        print("=== Control Motor + Encoder - Raspberry Pi ===")
        print("Comandos:")
        print("f = adelante, b = atrás, s = parar")
        print("+ = aumentar velocidad, - = disminuir velocidad")
        print("r = reset encoder, e = mostrar encoder")
        print("p<num> = ir a posición (ej: p1000)")
        print("t = test automático, q = salir")
        print("============================================")

    def setup_gpio(self):
        """Configurar pines GPIO"""
        GPIO.setmode(GPIO.BCM)
        GPIO.setwarnings(False)
        
        # Configurar encoder como entradas con pull-up
        GPIO.setup(ENCODER_A, GPIO.IN, pull_up_down=GPIO.PUD_UP)
        GPIO.setup(ENCODER_B, GPIO.IN, pull_up_down=GPIO.PUD_UP)
        
        # Configurar motor driver como salidas
        GPIO.setup(MOTOR_ENA, GPIO.OUT)
        GPIO.setup(MOTOR_IN1, GPIO.OUT)
        GPIO.setup(MOTOR_IN2, GPIO.OUT)
        
        # Configurar PWM para velocidad
        self.pwm = GPIO.PWM(MOTOR_ENA, 1000)  # 1000 Hz
        self.pwm.start(0)
        
        # Detener motor inicialmente
        self.stop_motor()

    def setup_encoder_interrupts(self):
        """Configurar interrupciones del encoder"""
        global last_a_state, last_b_state
        
        # Leer estados iniciales
        last_a_state = GPIO.input(ENCODER_A)
        last_b_state = GPIO.input(ENCODER_B)
        
        # Configurar interrupciones en flancos
        GPIO.add_event_detect(ENCODER_A, GPIO.BOTH, callback=self.encoder_callback, bouncetime=1)
        GPIO.add_event_detect(ENCODER_B, GPIO.BOTH, callback=self.encoder_callback, bouncetime=1)

    def encoder_callback(self, channel):
        """Callback para interrupciones del encoder"""
        global encoder_count, last_a_state, last_b_state
        
        with encoder_lock:
            a_state = GPIO.input(ENCODER_A)
            b_state = GPIO.input(ENCODER_B)
            
            # Detectar dirección usando lógica de cuadratura
            if channel == ENCODER_A:
                if a_state != last_a_state:
                    if a_state != b_state:
                        encoder_count += 1
                    else:
                        encoder_count -= 1
                    last_a_state = a_state
            
            elif channel == ENCODER_B:
                if b_state != last_b_state:
                    if a_state == b_state:
                        encoder_count += 1
                    else:
                        encoder_count -= 1
                    last_b_state = b_state

    def move_forward(self):
        """Mover motor hacia adelante"""
        GPIO.output(MOTOR_IN1, GPIO.HIGH)
        GPIO.output(MOTOR_IN2, GPIO.LOW)
        self.pwm.ChangeDutyCycle(motor_speed)
        print(f"Motor: ADELANTE - Velocidad: {motor_speed}%")

    def move_backward(self):
        """Mover motor hacia atrás"""
        GPIO.output(MOTOR_IN1, GPIO.LOW)
        GPIO.output(MOTOR_IN2, GPIO.HIGH)
        self.pwm.ChangeDutyCycle(motor_speed)
        print(f"Motor: ATRÁS - Velocidad: {motor_speed}%")

    def stop_motor(self):
        """Detener motor"""
        GPIO.output(MOTOR_IN1, GPIO.LOW)
        GPIO.output(MOTOR_IN2, GPIO.LOW)
        self.pwm.ChangeDutyCycle(0)
        print("Motor: PARADO")

    def set_speed(self, speed):
        """Establecer velocidad del motor (0-100)"""
        global motor_speed
        motor_speed = max(0, min(100, speed))
        self.pwm.ChangeDutyCycle(motor_speed)
        print(f"Velocidad: {motor_speed}%")

    def get_encoder_count(self):
        """Obtener cuenta actual del encoder"""
        with encoder_lock:
            return encoder_count

    def reset_encoder(self):
        """Resetear contador del encoder"""
        global encoder_count
        with encoder_lock:
            encoder_count = 0
        print("Encoder reseteado")

    def go_to_position(self, target):
        """Ir a una posición específica"""
        global target_position, position_control
        target_position = target
        position_control = True
        print(f"Yendo a posición: {target}")
        
        while position_control:
            current_pos = self.get_encoder_count()
            error = target_position - current_pos
            
            if abs(error) < 5:
                self.stop_motor()
                position_control = False
                print("¡Posición alcanzada!")
                break
            
            # Control proporcional simple
            speed = min(abs(error) * 2 + 30, 80)  # Velocidad mínima 30%, máxima 80%
            self.set_speed(speed)
            
            if error > 0:
                self.move_forward()
            else:
                self.move_backward()
            
            time.sleep(0.1)

    def test_motor(self):
        """Modo test automático"""
        print("=== INICIANDO TEST AUTOMÁTICO ===")
        initial_count = self.get_encoder_count()
        
        # Test 1: Adelante
        print("Test 1: Moviendo adelante por 3 segundos...")
        self.set_speed(60)
        self.move_forward()
        time.sleep(3)
        self.stop_motor()
        
        count_after_forward = self.get_encoder_count()
        print(f"Encoder cambió: {count_after_forward - initial_count}")
        
        time.sleep(1)
        
        # Test 2: Atrás
        print("Test 2: Moviendo atrás por 3 segundos...")
        self.move_backward()
        time.sleep(3)
        self.stop_motor()
        
        final_count = self.get_encoder_count()
        print(f"Encoder cambió: {final_count - count_after_forward}")
        print(f"Posición final vs inicial: {final_count - initial_count}")
        
        # Test 3: Control de posición
        print("Test 3: Control de posición...")
        self.reset_encoder()
        self.go_to_position(500)
        time.sleep(2)
        self.go_to_position(-300)
        time.sleep(2)
        self.go_to_position(0)
        
        print("=== TEST COMPLETADO ===")

    def handle_command(self, command):
        """Manejar comandos del usuario"""
        global position_control
        
        if command == 'f':
            position_control = False
            self.move_forward()
        
        elif command == 'b':
            position_control = False
            self.move_backward()
        
        elif command == 's':
            position_control = False
            self.stop_motor()
        
        elif command == '+':
            position_control = False
            self.set_speed(motor_speed + 10)
        
        elif command == '-':
            position_control = False
            self.set_speed(motor_speed - 10)
        
        elif command == 'r':
            self.reset_encoder()
        
        elif command == 'e':
            print(f"Encoder actual: {self.get_encoder_count()}")
        
        elif command.startswith('p'):
            try:
                pos = int(command[1:])
                self.go_to_position(pos)
            except ValueError:
                print("Formato inválido. Usar p<número> (ej: p1000)")
        
        elif command == 't':
            position_control = False
            self.test_motor()
        
        elif command == 'q':
            return False
        
        else:
            print("Comando no reconocido")
        
        return True

    def cleanup(self):
        """Limpiar recursos"""
        self.stop_motor()
        self.pwm.stop()
        GPIO.cleanup()
        print("GPIO limpiado. ¡Adiós!")

    def run(self):
        """Bucle principal"""
        try:
            while True:
                command = input("\nComando: ").strip().lower()
                if not self.handle_command(command):
                    break
                
                # Mostrar estado cada cierto tiempo
                if not position_control:
                    print(f"Estado - Encoder: {self.get_encoder_count()}, Velocidad: {motor_speed}%")
        
        except KeyboardInterrupt:
            print("\nInterrumpido por usuario")
        
        finally:
            self.cleanup()

if __name__ == "__main__":
    controller = MotorController()
    controller.run()