import RPi.GPIO as GPIO
import time

class RobotControl:
    def __init__(self):
        # Configuración GPIO
        GPIO.setmode(GPIO.BCM)
        GPIO.setwarnings(False)

        # Configuración de pines del motor
        self.ENA = 26
        self.IN1 = 19
        self.IN2 = 13
        self.ENB = 11
        self.IN3 = 6
        self.IN4 = 5

        self.setup_motores()

        # Velocidades
        self.velocidad_adelante = 60
        self.velocidad_atras = -40
        self.velocidad_giro = 50

    def setup_motores(self):
        # Configurar pines de motores
        motor_pins = [self.ENA, self.IN1, self.IN2, self.ENB, self.IN3, self.IN4]
        for pin in motor_pins:
            GPIO.setup(pin, GPIO.OUT)
            GPIO.output(pin, GPIO.LOW)

        # Configurar PWM
        self.pwm_a = GPIO.PWM(self.ENA, 1000)
        self.pwm_b = GPIO.PWM(self.ENB, 1000)
        self.pwm_a.start(0)
        self.pwm_b.start(0)

    def control_motores(self, izq, der):
        # Limitar velocidades
        izq = max(min(izq, 100), -100)
        der = max(min(der, 100), -100)

        # Control motor izquierdo
        GPIO.output(self.IN1, GPIO.HIGH if izq >= 0 else GPIO.LOW)
        GPIO.output(self.IN2, GPIO.LOW if izq >= 0 else GPIO.HIGH)
        self.pwm_a.ChangeDutyCycle(abs(izq))

        # Control motor derecho
        GPIO.output(self.IN3, GPIO.HIGH if der >= 0 else GPIO.LOW)
        GPIO.output(self.IN4, GPIO.LOW if der >= 0 else GPIO.HIGH)
        self.pwm_b.ChangeDutyCycle(abs(der))

    def realizar_rutina(self):
        try:
            # Rutina para hacer una "U"
            print("Realizando U")
            self.control_motores(self.velocidad_adelante, self.velocidad_adelante)
            time.sleep(2)
            self.control_motores(self.velocidad_giro, -self.velocidad_giro)
            time.sleep(1)
            self.control_motores(self.velocidad_adelante, self.velocidad_adelante)
            time.sleep(2)
            self.control_motores(-self.velocidad_giro, self.velocidad_giro)
            time.sleep(1)
            self.control_motores(self.velocidad_adelante, self.velocidad_adelante)
            time.sleep(2)

            # Rutina para hacer una "A"
            print("Realizando A")
            self.control_motores(self.velocidad_giro, -self.velocidad_giro)
            time.sleep(1)
            self.control_motores(self.velocidad_adelante, self.velocidad_adelante)
            time.sleep(2)
            self.control_motores(-self.velocidad_giro, self.velocidad_giro)
            time.sleep(1)
            self.control_motores(self.velocidad_adelante, self.velocidad_adelante)
            time.sleep(1)
            self.control_motores(self.velocidad_giro, -self.velocidad_giro)
            time.sleep(1)

            # Rutina para hacer una "B"
            print("Realizando B")
            self.control_motores(self.velocidad_adelante, self.velocidad_adelante)
            time.sleep(2)
            self.control_motores(self.velocidad_giro, -self.velocidad_giro)
            time.sleep(1)
            self.control_motores(self.velocidad_adelante, self.velocidad_adelante)
            time.sleep(1)
            self.control_motores(-self.velocidad_giro, self.velocidad_giro)
            time.sleep(1)
            self.control_motores(self.velocidad_adelante, self.velocidad_adelante)
            time.sleep(1)
            self.control_motores(self.velocidad_giro, -self.velocidad_giro)
            time.sleep(1)

            # Rutina para hacer una "C"
            print("Realizando C")
            self.control_motores(self.velocidad_giro, -self.velocidad_giro)
            time.sleep(2)
            self.control_motores(self.velocidad_adelante, self.velocidad_adelante)
            time.sleep(2)
            self.control_motores(-self.velocidad_giro, self.velocidad_giro)
            time.sleep(2)

        except KeyboardInterrupt:
            self.detener()

    def detener(self):
        self.control_motores(0, 0)
        self.pwm_a.stop()
        self.pwm_b.stop()
        GPIO.cleanup()
        print("\nSistema detenido correctamente")

if __name__ == "__main__":
    robot = RobotControl()
    print("Iniciando rutina de movimientos...")
    print("Presiona Ctrl+C para salir")
    robot.realizar_rutina()
