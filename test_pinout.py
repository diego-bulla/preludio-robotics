import RPi.GPIO as GPIO
import time

# Configuracion inicial
GPIO.setmode(GPIO.BCM)
GPIO.setwarnings(False)

# Definición de pines
LED_PINS = [17, 18, 27, 22, 23, 10, 9, 11]  # 8 LEDs
SERVO_PIN = 12  # Pin para el servomotor

# Configurar todos los pines LED como salida
for pin in LED_PINS:
    GPIO.setup(pin, GPIO.OUT)

# Configurar el servo
GPIO.setup(SERVO_PIN, GPIO.OUT)
servo = GPIO.PWM(SERVO_PIN, 50)  # 50Hz (período de 20ms)
servo.start(0)

def set_servo_angle(angle):
    """
    Convierte el ángulo (0-180) a duty cycle (2-12)
    y mueve el servo a esa posición
    """
    duty = angle / 18 + 2
    servo.ChangeDutyCycle(duty)
    time.sleep(0.3)  # Tiempo para que el servo alcance la posición

def control_leds():
    """Enciende todos los LEDs por 3 segundos y luego los apaga"""
    # Encender todos los LEDs
    for pin in LED_PINS:
        GPIO.output(pin, GPIO.HIGH)
    
    # Esperar 3 segundos
    time.sleep(3)
    
    # Apagar todos los LEDs
    for pin in LED_PINS:
        GPIO.output(pin, GPIO.LOW)

try:
    while True:
        # Rutina de LEDs
        print("Activando LEDs...")
        control_leds()
        
        # Rutina del servomotor
        print("Moviendo servomotor...")
        for angle in range(0, 181, 45):  # 0°, 45°, 90°, 135°, 180°
            print(f"Ángulo: {angle}°")
            set_servo_angle(angle)
            time.sleep(1)  # Esperar 1 segundo entre movimientos
        
        # Volver a la posición inicial
        set_servo_angle(0)
        time.sleep(1)

except KeyboardInterrupt:
    print("\nPrograma terminado por el usuario")
finally:
    servo.stop()
    GPIO.cleanup()
