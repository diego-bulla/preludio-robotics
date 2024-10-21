import RPi.GPIO as GPIO
import time

# Configuración de pines GPIO
GPIO.setmode(GPIO.BCM)

# Pines de los motores
int1 = 3
int2 = 24
ena_A = 26
int3 = 25
int4 = 2
ena_b = 13

# Pines de los LEDs
led1 = 23  # avanzar
led2 = 17  # retroceder
led3 = 18  # giro a la izquierda
led4 = 27  # giro a la derecha
led5 = 22  # parar

# Pines del sensor de ultrasonido
trig_pin = 5
echo_pin = 6

# Pines de los sensores infrarrojos para seguimiento de línea
ir_left = 16
ir_right = 20

# Configurar los pines como salida y entrada
GPIO.setup(int1, GPIO.OUT)
GPIO.setup(int2, GPIO.OUT)
GPIO.setup(ena_A, GPIO.OUT)
GPIO.setup(int3, GPIO.OUT)
GPIO.setup(int4, GPIO.OUT)
GPIO.setup(ena_b, GPIO.OUT)

GPIO.setup(led1, GPIO.OUT)
GPIO.setup(led2, GPIO.OUT)
GPIO.setup(led3, GPIO.OUT)
GPIO.setup(led4, GPIO.OUT)
GPIO.setup(led5, GPIO.OUT)

GPIO.setup(trig_pin, GPIO.OUT)
GPIO.setup(echo_pin, GPIO.IN)
GPIO.setup(ir_left, GPIO.IN)
GPIO.setup(ir_right, GPIO.IN)

# Configurar PWM para el control de velocidad (100% por defecto)
pwm_A = GPIO.PWM(ena_A, 1000)  # Frecuencia de 1 kHz
pwm_B = GPIO.PWM(ena_b, 1000)  # Frecuencia de 1 kHz
pwm_A.start(100)  # Velocidad al 100%
pwm_B.start(100)

# Función para medir la distancia usando el sensor de ultrasonido
def get_distance():
    # Enviar un pulso ultrasonico
    GPIO.output(trig_pin, GPIO.HIGH)
    time.sleep(0.00001)
    GPIO.output(trig_pin, GPIO.LOW)

    # Medir el tiempo que tarda el pulso en regresar
    while GPIO.input(echo_pin) == GPIO.LOW:
        start_time = time.time()

    while GPIO.input(echo_pin) == GPIO.HIGH:
        end_time = time.time()

    # Calcular la distancia en centímetros
    elapsed_time = end_time - start_time
    distance = (elapsed_time * 34300) / 2  # Velocidad del sonido: 34300 cm/s

    return distance

# Funciones de movimiento del vehículo
def forward():
    reset_leds()
    GPIO.output(int1, GPIO.HIGH)
    GPIO.output(int2, GPIO.LOW)
    GPIO.output(int3, GPIO.HIGH)
    GPIO.output(int4, GPIO.LOW)
    GPIO.output(led1, GPIO.HIGH)
    print("Avanzando")

def backward():
    reset_leds()
    GPIO.output(int1, GPIO.LOW)
    GPIO.output(int2, GPIO.HIGH)
    GPIO.output(int3, GPIO.LOW)
    GPIO.output(int4, GPIO.HIGH)
    GPIO.output(led2, GPIO.HIGH)
    print("Retrocediendo")

def left():
    reset_leds()
    GPIO.output(int1, GPIO.LOW)
    GPIO.output(int2, GPIO.HIGH)
    GPIO.output(int3, GPIO.HIGH)
    GPIO.output(int4, GPIO.LOW)
    GPIO.output(led3, GPIO.HIGH)
    print("Girando a la izquierda")

def right():
    reset_leds()
    GPIO.output(int1, GPIO.HIGH)
    GPIO.output(int2, GPIO.LOW)
    GPIO.output(int3, GPIO.LOW)
    GPIO.output(int4, GPIO.HIGH)
    GPIO.output(led4, GPIO.HIGH)
    print("Girando a la derecha")

def stop():
    GPIO.output(ena_A, GPIO.LOW)
    GPIO.output(ena_b, GPIO.LOW)
    reset_leds()
    GPIO.output(led5, GPIO.HIGH)
    print("Parado")

# Función para resetear los LEDs
def reset_leds():
    GPIO.output(led1, GPIO.LOW)
    GPIO.output(led2, GPIO.LOW)
    GPIO.output(led3, GPIO.LOW)
    GPIO.output(led4, GPIO.LOW)
    GPIO.output(led5, GPIO.LOW)

# Función para seguir la línea negra
def follow_line():
    left_sensor = GPIO.input(ir_left)
    right_sensor = GPIO.input(ir_right)

    if left_sensor == GPIO.LOW and right_sensor == GPIO.LOW:
        # Ambos sensores detectan la línea, avanzar recto
        forward()
    elif left_sensor == GPIO.LOW and right_sensor == GPIO.HIGH:
        # Solo el sensor izquierdo detecta la línea, girar a la izquierda
        left()
    elif left_sensor == GPIO.HIGH and right_sensor == GPIO.LOW:
        # Solo el sensor derecho detecta la línea, girar a la derecha
        right()
    else:
        # Ningún sensor detecta la línea, detenerse
        stop()

# Rutina principal
try:
    while True:
        # Verificar la distancia del sensor de ultrasonido
        distance = get_distance()
        if distance < 20:  # Si el obstáculo está a menos de 20 cm
            print("Obstáculo detectado a {:.2f} cm".format(distance))
            stop()
        else:
            # Seguir la línea mientras no haya obstáculos
            follow_line()

        time.sleep(0.1)  # Esperar 100 ms entre iteraciones

finally:
    # Detener los motores y limpiar los GPIO al finalizar
    pwm_A.stop()
    pwm_B.stop()
    GPIO.cleanup()
