import RPi.GPIO as GPIO
import time

class RobotSeguidorLinea:
    def __init__(self):
        GPIO.setmode(GPIO.BCM)
        GPIO.setwarnings(False)
        
        # Configuración de pines (¡verifica conexiones físicas!)
        self.SENSOR_PINS = [24, 25, 8, 7, 12, 16, 20, 21]  # Pines QTR-8RC
        self.ENA = 26    # PWM Motor A
        self.IN1 = 19    # Control Motor A
        self.IN2 = 13
        self.ENB = 11    # PWM Motor B
        self.IN3 = 6     # Control Motor B
        self.IN4 = 5
        
        self.setup_pins()
        
        # Parámetros ajustables (¡calibra estos valores!)
        self.velocidad_base = 70   # 70-80 funciona mejor para motores comunes
        self.tiempo_lectura = 0.1
        self.umbral = 0.003        # 0.001 (línea negra) a 0.01 (blanco)
        
        # PID inicial (ajusta en marcha)
        self.Kp = 1.5    # Proporcional: controla la agresividad del giro
        self.Ki = 0.0    # Integral: desactívalo al inicio (0)
        self.Kd = 0.0    # Derivativo: desactívalo al inicio (0)
        
        self.error_anterior = 0
        self.integral = 0

    def setup_pins(self):
        # 1. Configurar motores
        motor_pins = [self.ENA, self.IN1, self.IN2, self.ENB, self.IN3, self.IN4]
        for pin in motor_pins:
            GPIO.setup(pin, GPIO.OUT)
            GPIO.output(pin, GPIO.LOW)
        
        # 2. Iniciar PWM
        self.pwm_a = GPIO.PWM(self.ENA, 1000)
        self.pwm_b = GPIO.PWM(self.ENB, 1000)
        self.pwm_a.start(0)
        self.pwm_b.start(0)
        
        # 3. Sensores con pull-down
        for pin in self.SENSOR_PINS:
            GPIO.setup(pin, GPIO.IN, pull_up_down=GPIO.PUD_DOWN)

    def leer_sensores(self):
        valores = []
        for pin in self.SENSOR_PINS:
            GPIO.setup(pin, GPIO.OUT)
            GPIO.output(pin, GPIO.HIGH)
            time.sleep(0.00001)  # 10 µs
            
            GPIO.setup(pin, GPIO.IN)
            inicio = time.time()
            while GPIO.input(pin) and (time.time() - inicio < self.tiempo_lectura):
                pass
            
            duracion = time.time() - inicio
            valores.append(duracion > self.umbral)  # True = línea negra detectada
        return valores

    def calcular_posicion(self, valores):
        if not any(valores):
            return None
            
        total = sum(i * 1000 if val else 0 for i, val in enumerate(valores))
        count = sum(1 for val in valores if val)
        
        return total / count if count != 0 else 3500  # Centro = 3500

    def control_pid(self, posicion):
        if posicion is None:
            return 0
            
        error = posicion - 3500  # Error respecto al centro
        P = self.Kp * error
        I = self.Ki * self.integral
        D = self.Kd * (error - self.error_anterior)
        
        self.integral += error
        self.integral = max(min(self.integral, 1000), -1000)
        self.error_anterior = error
        
        return P + I + D

    def control_motores(self, izq, der):
        izq = max(min(izq, 100), -100)
        der = max(min(der, 100), -100)
        
        # Motor izquierdo
        GPIO.output(self.IN1, GPIO.HIGH if izq >=0 else GPIO.LOW)
        GPIO.output(self.IN2, GPIO.LOW if izq >=0 else GPIO.HIGH)
        self.pwm_a.ChangeDutyCycle(abs(izq))
        
        # Motor derecho
        GPIO.output(self.IN3, GPIO.HIGH if der >=0 else GPIO.LOW)
        GPIO.output(self.IN4, GPIO.LOW if der >=0 else GPIO.HIGH)
        self.pwm_b.ChangeDutyCycle(abs(der))

    def seguir_linea(self):
        print("\n🚀 Modo seguimiento activado! (Ctrl+C para salir)")
        print("► Posición esperada: 3500 (centro)")
        print("► Ajuste PID positivo = Giro a la derecha")
        print("► Ajuste PID negativo = Giro a la izquierda\n")
        
        try:
            while True:
                valores = self.leer_sensores()
                posicion = self.calcular_posicion(valores)
                
                if posicion is not None:
                    ajuste = self.control_pid(posicion)
                    vel_izq = self.velocidad_base - ajuste
                    vel_der = self.velocidad_base + ajuste
                    
                    # Mensajes de diagnóstico
                    print(f"Sensores: {[1 if v else 0 for v in valores]}")
                    print(f"Posición: {posicion:.0f} | Ajuste PID: {ajuste:.1f}")
                    print(f"Motores: I={vel_izq:.1f}% | D={vel_der:.1f}%")
                    print("---------------------------------------")
                    
                    self.control_motores(vel_izq, vel_der)
                else:
                    print("¡LÍNEA PERDIDA! Deteniendo motores...")
                    self.control_motores(0, 0)
                
                time.sleep(0.05)
        except KeyboardInterrupt:
            self.detener()

    def prueba_motores(self):
        print("\n🔧 Modo prueba de motores:")
        secuencia = [
            ("Adelante Izquierdo", 50, 0),
            ("Adelante Derecho", 0, 50),
            ("Ambos Adelante", 70, 70),
            ("Ambos Atrás", -50, -50),
            ("Giro Derecha", 50, -50),
            ("Giro Izquierda", -50, 50)
        ]
        
        for nombre, izq, der in secuencia:
            print(f"\n► {nombre}")
            self.control_motores(izq, der)
            time.sleep(1.5)
        
        self.detener()
        print("\nPrueba completada ✔️")

    def prueba_sensores(self):
        print("\n📡 Modo monitor de sensores (Ctrl+C para salir):")
        try:
            while True:
                valores = self.leer_sensores()
                print(" ".join(["■" if v else "□" for v in valores]), end="\r")
                time.sleep(0.1)
        except KeyboardInterrupt:
            print("\nMonitor detenido ✔️")

    def detener(self):
        self.control_motores(0, 0)
        self.pwm_a.stop()
        self.pwm_b.stop()

if __name__ == "__main__":
    robot = RobotSeguidorLinea()
    try:
        while True:
            print("\n=== MENÚ PRINCIPAL ===")
            print("1. Iniciar seguimiento de línea")
            print("2. Probar motores")
            print("3. Monitor de sensores (RAW)")
            print("4. Salir")
            
            opcion = input("Seleccione una opción: ")
            
            if opcion == "1":
                robot.seguir_linea()
            elif opcion == "2":
                robot.prueba_motores()
            elif opcion == "3":
                robot.prueba_sensores()
            elif opcion == "4":
                print("\n¡Hasta luego! 🤖")
                break
            else:
                print("Opción no válida ❌")
    finally:
        GPIO.cleanup()
