#!/usr/bin/env python3
# motor_control.py - Control de Motores y Encoders

import RPi.GPIO as GPIO
import time
import threading
from config import *

class MotorController:
    def __init__(self):
        # Configurar GPIO
        GPIO.setmode(GPIO.BCM)
        GPIO.setwarnings(False)
        
        # Configurar pines de motores
        motor_pins = [MOTOR_LEFT_IN1, MOTOR_LEFT_IN2, MOTOR_RIGHT_IN3, MOTOR_RIGHT_IN4]
        for pin in motor_pins:
            GPIO.setup(pin, GPIO.OUT)
            GPIO.output(pin, GPIO.LOW)
        
        # Configurar PWM
        GPIO.setup(MOTOR_LEFT_PWM, GPIO.OUT)
        GPIO.setup(MOTOR_RIGHT_PWM, GPIO.OUT)
        
        self.pwm_left = GPIO.PWM(MOTOR_LEFT_PWM, PWM_FREQUENCY)
        self.pwm_right = GPIO.PWM(MOTOR_RIGHT_PWM, PWM_FREQUENCY)
        
        self.pwm_left.start(0)
        self.pwm_right.start(0)
        
        # Variables de estado
        self.running = True
        
        # Configurar encoders
        self._setup_encoders()
        
        print("✅ MotorController inicializado")
    
    def _setup_encoders(self):
        """Configurar encoders con interrupciones"""
        # Configurar pines de encoder como entrada con pull-up
        encoder_pins = [ENCODER_LEFT_A, ENCODER_LEFT_B, ENCODER_RIGHT_A, ENCODER_RIGHT_B]
        for pin in encoder_pins:
            GPIO.setup(pin, GPIO.IN, pull_up_down=GPIO.PUD_UP)
        
        # Contadores de pulsos
        self.left_pulses = 0
        self.right_pulses = 0
        
        # Lock para thread safety
        self.encoder_lock = threading.Lock()
        
        # Configurar interrupciones
        GPIO.add_event_detect(ENCODER_LEFT_A, GPIO.RISING, callback=self._left_encoder_callback)
        GPIO.add_event_detect(ENCODER_RIGHT_A, GPIO.RISING, callback=self._right_encoder_callback)
        
        print("✅ Encoders configurados")
    
    def _left_encoder_callback(self, channel):
        """Callback para encoder izquierdo"""
        with self.encoder_lock:
            self.left_pulses += 1
    
    def _right_encoder_callback(self, channel):
        """Callback para encoder derecho"""
        with self.encoder_lock:
            self.right_pulses += 1
    
    def get_encoder_counts(self):
        """Obtener conteos actuales de encoders"""
        with self.encoder_lock:
            return self.left_pulses, self.right_pulses
    
    def reset_encoders(self):
        """Reiniciar contadores de encoders"""
        with self.encoder_lock:
            self.left_pulses = 0
            self.right_pulses = 0
        print("🔄 Encoders reiniciados")
    
    def set_motor_speed(self, left_speed, right_speed):
        """
        Establecer velocidad de motores
        left_speed, right_speed: -100 a 100 (negativo = reversa)
        """
        # Limitar velocidades
        left_speed = max(-100, min(100, left_speed))
        right_speed = max(-100, min(100, right_speed))
        
        # Motor izquierdo
        if left_speed > 0:
            GPIO.output(MOTOR_LEFT_IN1, GPIO.HIGH)
            GPIO.output(MOTOR_LEFT_IN2, GPIO.LOW)
            self.pwm_left.ChangeDutyCycle(abs(left_speed))
        elif left_speed < 0:
            GPIO.output(MOTOR_LEFT_IN1, GPIO.LOW)
            GPIO.output(MOTOR_LEFT_IN2, GPIO.HIGH)
            self.pwm_left.ChangeDutyCycle(abs(left_speed))
        else:
            GPIO.output(MOTOR_LEFT_IN1, GPIO.LOW)
            GPIO.output(MOTOR_LEFT_IN2, GPIO.LOW)
            self.pwm_left.ChangeDutyCycle(0)
        
        # Motor derecho
        if right_speed > 0:
            GPIO.output(MOTOR_RIGHT_IN3, GPIO.HIGH)
            GPIO.output(MOTOR_RIGHT_IN4, GPIO.LOW)
            self.pwm_right.ChangeDutyCycle(abs(right_speed))
        elif right_speed < 0:
            GPIO.output(MOTOR_RIGHT_IN3, GPIO.LOW)
            GPIO.output(MOTOR_RIGHT_IN4, GPIO.HIGH)
            self.pwm_right.ChangeDutyCycle(abs(right_speed))
        else:
            GPIO.output(MOTOR_RIGHT_IN3, GPIO.LOW)
            GPIO.output(MOTOR_RIGHT_IN4, GPIO.LOW)
            self.pwm_right.ChangeDutyCycle(0)
    
    def stop_motors(self):
        """Detener ambos motores"""
        self.set_motor_speed(0, 0)
        print("🛑 Motores detenidos")
    
    def move_forward(self, speed=TEST_SPEED):
        """Mover hacia adelante - motores giran en direcciones opuestas"""
        # Motor izquierdo antihorario (+), motor derecho horario (-)
        self.set_motor_speed(speed, -speed)
        print(f"⬆️  Adelante - Velocidad: {speed}%")
    
    def move_backward(self, speed=TEST_SPEED):
        """Mover hacia atrás - invertir direcciones"""
        # Motor izquierdo horario (-), motor derecho antihorario (+)
        self.set_motor_speed(-speed, speed)
        print(f"⬇️  Atrás - Velocidad: {speed}%")
    
    def turn_left(self, speed=TEST_SPEED):
        """Girar a la izquierda - ambos motores hacia adelante con diferente velocidad"""
        # Motor izquierdo más lento, motor derecho más rápido
        self.set_motor_speed(speed//3, -speed)
        print(f"⬅️  Izquierda - Velocidad: {speed}%")
    
    def turn_right(self, speed=TEST_SPEED):
        """Girar a la derecha - ambos motores hacia adelante con diferente velocidad"""
        # Motor izquierdo más rápido, motor derecho más lento
        self.set_motor_speed(speed, -speed//3)
        print(f"➡️  Derecha - Velocidad: {speed}%")
    
    def spin_left(self, speed=TEST_SPEED):
        """Girar en su lugar hacia la izquierda"""
        # Motor izquierdo hacia atrás, motor derecho hacia adelante
        self.set_motor_speed(-speed, -speed)
        print(f"🔄 Giro en sitio izquierda - Velocidad: {speed}%")
    
    def spin_right(self, speed=TEST_SPEED):
        """Girar en su lugar hacia la derecha"""
        # Motor izquierdo hacia adelante, motor derecho hacia atrás
        self.set_motor_speed(speed, speed)
        print(f"🔄 Giro en sitio derecha - Velocidad: {speed}%")
    
    def get_distance_traveled(self):
        """Calcular distancia recorrida por cada rueda"""
        left_count, right_count = self.get_encoder_counts()
        
        # Convertir pulsos a distancia
        left_distance = (left_count / PULSES_PER_REVOLUTION) * WHEEL_CIRCUMFERENCE_CM
        right_distance = (right_count / PULSES_PER_REVOLUTION) * WHEEL_CIRCUMFERENCE_CM
        
        return left_distance, right_distance
    
    def get_average_distance(self):
        """Obtener distancia promedio (centro del robot)"""
        left_dist, right_dist = self.get_distance_traveled()
        return (left_dist + right_dist) / 2.0
    
    def get_robot_position(self):
        """Calcular posición aproximada del robot (odometría simple)"""
        left_dist, right_dist = self.get_distance_traveled()
        
        # Distancia del centro del robot
        center_distance = (left_dist + right_dist) / 2.0
        
        # Ángulo de rotación aproximado
        distance_diff = right_dist - left_dist
        angle_radians = distance_diff / WHEELBASE_CM
        
        return center_distance, angle_radians
    
    def cleanup(self):
        """Limpiar recursos GPIO"""
        self.running = False
        self.stop_motors()
        self.pwm_left.stop()
        self.pwm_right.stop()
        GPIO.cleanup()
        print("🧹 GPIO limpiado")

if __name__ == "__main__":
    # Prueba básica del motor controller
    try:
        motors = MotorController()
        
        print("🧪 Iniciando prueba básica de motores...")
        
        # Prueba de movimiento hacia adelante
        print("Testing forward movement...")
        motors.move_forward(30)
        time.sleep(2)
        motors.stop_motors()
        time.sleep(1)
        
        # Mostrar datos de encoders
        left_dist, right_dist = motors.get_distance_traveled()
        print(f"Distancia - Izq: {left_dist:.2f}cm, Der: {right_dist:.2f}cm")
        
        motors.cleanup()
        
    except KeyboardInterrupt:
        print("\n❌ Prueba interrumpida")
        motors.cleanup()
    except Exception as e:
        print(f"❌ Error: {e}")
        motors.cleanup()