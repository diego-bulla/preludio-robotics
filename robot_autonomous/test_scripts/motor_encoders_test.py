#!/usr/bin/env python3
# test_simple_motores_encoders.py - Prueba simple de motores + encoders

import RPi.GPIO as GPIO
import time
import threading

# Configuración de pines
# Motores L298N
ENA = 13    # Enable Motor A (Derecho)
IN1 = 19    # Motor A dirección 1
IN2 = 16    # Motor A dirección 2
IN3 = 26    # Motor B dirección 1 (Izquierdo)
IN4 = 20    # Motor B dirección 2
ENB = 21    # Enable Motor B

# Encoders
ENCODER_A_RIGHT = 27  # Encoder derecho canal A
ENCODER_B_RIGHT = 17  # Encoder derecho canal B
ENCODER_A_LEFT = 6    # Encoder izquierdo canal A
ENCODER_B_LEFT = 5    # Encoder izquierdo canal B

# Variables globales para conteo
count_right = 0
count_left = 0
running = True

# Configurar GPIO
GPIO.cleanup()
GPIO.setmode(GPIO.BCM)
GPIO.setwarnings(False)

# Configurar pines de motores
GPIO.setup([ENA, IN1, IN2, IN3, IN4, ENB], GPIO.OUT)

# Configurar pines de encoders
GPIO.setup([ENCODER_A_RIGHT, ENCODER_B_RIGHT, ENCODER_A_LEFT, ENCODER_B_LEFT], 
           GPIO.IN, pull_up_down=GPIO.PUD_UP)

# Configurar PWM
pwm_right = GPIO.PWM(ENA, 1000)
pwm_left = GPIO.PWM(ENB, 1000)
pwm_right.start(0)
pwm_left.start(0)

def encoder_callback_right(channel):
    """Callback para encoder derecho"""
    global count_right
    count_right += 1

def encoder_callback_left(channel):
    """Callback para encoder izquierdo"""
    global count_left
    count_left += 1

def setup_encoders():
    """Configurar interrupciones de encoders"""
    GPIO.add_event_detect(ENCODER_A_RIGHT, GPIO.RISING, callback=encoder_callback_right)
    GPIO.add_event_detect(ENCODER_A_LEFT, GPIO.RISING, callback=encoder_callback_left)
    print("📡 Encoders configurados")

def stop_motors():
    """Parar ambos motores"""
    pwm_right.ChangeDutyCycle(0)
    pwm_left.ChangeDutyCycle(0)
    GPIO.output([IN1, IN2, IN3, IN4], GPIO.LOW)

def move_forward(speed=50, duration=2):
    """Mover adelante"""
    global count_right, count_left
    print(f"🔄 Adelante - Velocidad: {speed}% - Duración: {duration}s")
    
    # Resetear contadores
    count_right = 0
    count_left = 0
    
    # Mover adelante
    GPIO.output(IN1, GPIO.HIGH)
    GPIO.output(IN2, GPIO.LOW)
    GPIO.output(IN3, GPIO.HIGH)
    GPIO.output(IN4, GPIO.LOW)
    pwm_right.ChangeDutyCycle(speed)
    pwm_left.ChangeDutyCycle(speed)
    
    # Esperar y mostrar conteo
    start_time = time.time()
    while time.time() - start_time < duration:
        print(f"   Encoders - Derecho: {count_right:4d} | Izquierdo: {count_left:4d}", end='\r')
        time.sleep(0.1)
    
    stop_motors()
    print(f"\n✅ Final - Derecho: {count_right} | Izquierdo: {count_left}")

def move_backward(speed=50, duration=2):
    """Mover atrás"""
    global count_right, count_left
    print(f"🔄 Atrás - Velocidad: {speed}% - Duración: {duration}s")
    
    count_right = 0
    count_left = 0
    
    # Mover atrás
    GPIO.output(IN1, GPIO.LOW)
    GPIO.output(IN2, GPIO.HIGH)
    GPIO.output(IN3, GPIO.LOW)
    GPIO.output(IN4, GPIO.HIGH)
    pwm_right.ChangeDutyCycle(speed)
    pwm_left.ChangeDutyCycle(speed)
    
    start_time = time.time()
    while time.time() - start_time < duration:
        print(f"   Encoders - Derecho: {count_right:4d} | Izquierdo: {count_left:4d}", end='\r')
        time.sleep(0.1)
    
    stop_motors()
    print(f"\n✅ Final - Derecho: {count_right} | Izquierdo: {count_left}")

def turn_left(speed=50, duration=1):
    """Girar izquierda"""
    global count_right, count_left
    print(f"🔄 Izquierda - Velocidad: {speed}% - Duración: {duration}s")
    
    count_right = 0
    count_left = 0
    
    # Motor derecho adelante, izquierdo atrás
    GPIO.output(IN1, GPIO.HIGH)
    GPIO.output(IN2, GPIO.LOW)
    GPIO.output(IN3, GPIO.LOW)
    GPIO.output(IN4, GPIO.HIGH)
    pwm_right.ChangeDutyCycle(speed)
    pwm_left.ChangeDutyCycle(speed)
    
    start_time = time.time()
    while time.time() - start_time < duration:
        print(f"   Encoders - Derecho: {count_right:4d} | Izquierdo: {count_left:4d}", end='\r')
        time.sleep(0.1)
    
    stop_motors()
    print(f"\n✅ Final - Derecho: {count_right} | Izquierdo: {count_left}")

def turn_right(speed=50, duration=1):
    """Girar derecha"""
    global count_right, count_left
    print(f"🔄 Derecha - Velocidad: {speed}% - Duración: {duration}s")
    
    count_right = 0
    count_left = 0
    
    # Motor derecho atrás, izquierdo adelante
    GPIO.output(IN1, GPIO.LOW)
    GPIO.output(IN2, GPIO.HIGH)
    GPIO.output(IN3, GPIO.HIGH)
    GPIO.output(IN4, GPIO.LOW)
    pwm_right.ChangeDutyCycle(speed)
    pwm_left.ChangeDutyCycle(speed)
    
    start_time = time.time()
    while time.time() - start_time < duration:
        print(f"   Encoders - Derecho: {count_right:4d} | Izquierdo: {count_left:4d}", end='\r')
        time.sleep(0.1)
    
    stop_motors()
    print(f"\n✅ Final - Derecho: {count_right} | Izquierdo: {count_left}")

def test_encoders_only():
    """Solo probar lectura de encoders sin mover motores"""
    global count_right, count_left
    print("📡 PRUEBA SOLO ENCODERS")
    print("Gira las ruedas manualmente...")
    
    count_right = 0
    count_left = 0
    
    for i in range(20):  # 10 segundos
        print(f"   Encoders - Derecho: {count_right:4d} | Izquierdo: {count_left:4d}", end='\r')
        time.sleep(0.5)
    
    print(f"\n✅ Final - Derecho: {count_right} | Izquierdo: {count_left}")

def test_motor_only():
    """Solo probar motores sin encoders"""
    print("🔄 PRUEBA SOLO MOTORES (sin encoders)")
    
    # Adelante
    print("Adelante 2 segundos...")
    GPIO.output(IN1, GPIO.HIGH)
    GPIO.output(IN2, GPIO.LOW)
    GPIO.output(IN3, GPIO.HIGH)
    GPIO.output(IN4, GPIO.LOW)
    pwm_right.ChangeDutyCycle(50)
    pwm_left.ChangeDutyCycle(50)
    time.sleep(2)
    
    stop_motors()
    time.sleep(1)
    
    print("✅ Motores funcionando")

def test_complete():
    """Prueba completa de motores + encoders"""
    print("🤖 PRUEBA COMPLETA - MOTORES + ENCODERS")
    print("=" * 50)
    
    try:
        # Configurar encoders
        setup_encoders()
        time.sleep(0.5)
        
        # Secuencia de pruebas
        print("\n1. Movimiento adelante:")
        move_forward(40, 3)
        time.sleep(1)
        
        print("\n2. Movimiento atrás:")
        move_backward(40, 3)
        time.sleep(1)
        
        print("\n3. Giro izquierda:")
        turn_left(40, 1.5)
        time.sleep(1)
        
        print("\n4. Giro derecha:")
        turn_right(40, 1.5)
        time.sleep(1)
        
        print("\n✅ PRUEBA COMPLETA TERMINADA")
        
    except Exception as e:
        print(f"\n❌ Error: {e}")

def cleanup():
    """Limpiar recursos"""
    global running
    running = False
    stop_motors()
    pwm_right.stop()
    pwm_left.stop()
    GPIO.cleanup()
    print("\n🧹 Limpieza completada")

def main():
    print("🤖 PRUEBA SIMPLE DE MOTORES + ENCODERS")
    print("=" * 50)
    print("Opciones:")
    print("1. Prueba solo encoders (manual)")
    print("2. Prueba solo motores")
    print("3. Prueba completa (motores + encoders)")
    print("4. Solo parar motores")
    
    try:
        choice = input("\nElige opción (1-4): ").strip()
        
        if choice == '1':
            setup_encoders()
            test_encoders_only()
        elif choice == '2':
            test_motor_only()
        elif choice == '3':
            print("\n⚠️  ASEGÚRATE DE QUE EL ROBOT ESTÉ EN EL SUELO")
            input("Presiona ENTER para continuar...")
            test_complete()
        elif choice == '4':
            print("🛑 Parando motores...")
            stop_motors()
            print("✅ Motores parados")
        else:
            print("❌ Opción inválida")
    
    except KeyboardInterrupt:
        print("\n❌ Interrumpido por usuario")
    
    except Exception as e:
        print(f"\n❌ Error: {e}")
    
    finally:
        cleanup()

if __name__ == "__main__":
    main()