import RPi.GPIO as GPIO
import cv2
import numpy as np
import time
# ===================
# CONFIGURACIÓN HARDWARE
# ===================
# Pines del Motor L298N
IN1 = 19
IN2 = 13
IN3 = 6
IN4 = 5
ENA = 26
ENB = 11
# Configuración de la cámara
CAMERA_WIDTH = 640
CAMERA_HEIGHT = 480
# Rango de colores (HSV) para verde y rojo
COLOR_RANGES = {
    "verde": [(40, 50, 50), (80, 255, 255)],  # Ajusta según iluminación
    "rojo": [(0, 50, 50), (10, 255, 255), (170, 50, 50), (180, 255, 255)]
}
# Velocidad de los motores
MAX_SPEED = 60  # Velocidad base (0-100%)
# ===================
# FUNCIONES PRINCIPALES
# ===================
def setup():
    GPIO.setmode(GPIO.BCM)
    GPIO.setup([IN1, IN2, IN3, IN4, ENA, ENB], GPIO.OUT)
    global pwm_a, pwm_b
    pwm_a = GPIO.PWM(ENA, 1000)
    pwm_b = GPIO.PWM(ENB, 1000)
    pwm_a.start(0)
    pwm_b.start(0)
def detectar_color(frame, color):
    """Detecta el color en el frame y devuelve el centro del objeto."""
    hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)
    if color == "rojo":
        mask1 = cv2.inRange(hsv, np.array(COLOR_RANGES["rojo"][0]), np.array(COLOR_RANGES["rojo"][1]))
        mask2 = cv2.inRange(hsv, np.array(COLOR_RANGES["rojo"][2]), np.array(COLOR_RANGES["rojo"][3]))
        mask = cv2.bitwise_or(mask1, mask2)
    else:
        mask = cv2.inRange(hsv, np.array(COLOR_RANGES[color][0]), np.array(COLOR_RANGES[color][1]))
    mask = cv2.erode(mask, None, iterations=2)
    mask = cv2.dilate(mask, None, iterations=2)
    contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
    if contours:
        largest_contour = max(contours, key=cv2.contourArea)
        if cv2.contourArea(largest_contour) > 500:  # Ajustar tamaño mínimo
            M = cv2.moments(largest_contour)
            if M["m00"] > 0:
                cx = int(M["m10"] / M["m00"])
                cy = int(M["m01"] / M["m00"])
                return cx, cy
    return None
def mover_motores(cx, frame_width):
    """Controla los motores para mover hacia el objeto."""
    error = cx - frame_width // 2
    correction = error / (frame_width // 2) * MAX_SPEED
    left_speed = MAX_SPEED - correction
    right_speed = MAX_SPEED + correction
    # Ajustar velocidad de motores
    GPIO.output(IN1, GPIO.HIGH if left_speed > 0 else GPIO.LOW)
    GPIO.output(IN2, GPIO.LOW if left_speed > 0 else GPIO.HIGH)
    pwm_a.ChangeDutyCycle(min(MAX_SPEED, abs(left_speed)))
    GPIO.output(IN3, GPIO.HIGH if right_speed > 0 else GPIO.LOW)
    GPIO.output(IN4, GPIO.LOW if right_speed > 0 else GPIO.HIGH)
    pwm_b.ChangeDutyCycle(min(MAX_SPEED, abs(right_speed)))
# ===================
# PROGRAMA PRINCIPAL
# ===================
def main():
    setup()
    cap = cv2.VideoCapture(0)
    cap.set(cv2.CAP_PROP_FRAME_WIDTH, CAMERA_WIDTH)
    cap.set(cv2.CAP_PROP_FRAME_HEIGHT, CAMERA_HEIGHT)
    try:
        while True:
            ret, frame = cap.read()
            if not ret:
                break
            # Detectar colores
            for color in COLOR_RANGES.keys():
                center = detectar_color(frame, color)
                if center:
                    cx, cy = center
                    print(f"Color detectado: {color} en posición {cx}, {cy}")
                    mover_motores(cx, CAMERA_WIDTH)
                    break
            else:
                # Detener motores si no se detecta color
                pwm_a.ChangeDutyCycle(0)
                pwm_b.ChangeDutyCycle(0)
            # Mostrar imagen (opcional)
            cv2.imshow("Detección de color", frame)
            if cv2.waitKey(1) == ord("q"):
                break
    except KeyboardInterrupt:
        pass
    finally:
        cap.release()
        cv2.destroyAllWindows()
        GPIO.cleanup()
if __name__ == "__main__":
    main()
