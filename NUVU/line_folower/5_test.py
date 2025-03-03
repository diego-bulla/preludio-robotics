import RPi.GPIO as GPIO
import time

class RobotSeguidorLinea:
    def __init__(self):
        # Configuración inicial GPIO
        GPIO.setmode(GPIO.BCM)
        GPIO.setwarnings(False)
        
        # Pines para QTR-8RC
        self.SENSOR_PINS = [24, 25, 8, 7, 12, 16, 20, 21]  # Ajusta según conexiones
        
        # Pines para L298N
        self.ENA = 26  # PWM motor A
        self.IN1 = 19  # Control motor A
        self.IN2 = 13
        self.ENB = 11  # PWM motor B
        self.IN3 = 6   # Control motor B
        self.IN4 = 5
        
        # Configurar pines
        self.setup_pins()
        
        # Parámetros de control
        self.velocidad_base = 50   # Rango 0-100
        self.tiempo_lectura = 0.1  # Tiempo de muestreo (segundos)
        self.umbral = 0.005        # Umbral para línea negra (ajusta según calibración)
        
        # Parámetros PID
        self.Kp = 0.5
        self.Ki = 0.0001
        self.Kd = 0.1
        self.error_anterior = 0
        self.integral = 0

    def setup_pins(self):
        """Configuración robusta de pines"""
        # 1. Configurar motores primero
        motor_pins = [self.ENA, self.IN1, self.IN2, self.ENB, self.IN3, self.IN4]
        for pin in motor_pins:
            GPIO.setup(pin, GPIO.OUT)
            GPIO.output(pin, GPIO.LOW)  # Inicializar en bajo
        
        # 2. Iniciar PWM
        self.pwm_a = GPIO.PWM(self.ENA, 1000)  # Frecuencia 1 kHz
        self.pwm_b = GPIO.PWM(self.ENB, 1000)
        self.pwm_a.start(0)
        self.pwm_b.start(0)
        
        # 3. Configurar sensores con pull-down
        for pin in self.SENSOR_PINS:
            GPIO.setup(pin, GPIO.IN, pull_up_down=GPIO.PUD_DOWN)  # Evita estado flotante

    def leer_sensores(self):
        """Lectura precisa de sensores QTR-8RC"""
        valores = []
        for pin in self.SENSOR_PINS:
            # Carga del capacitor
            GPIO.setup(pin, GPIO.OUT)
            GPIO.output(pin, GPIO.HIGH)
            time.sleep(0.00001)  # 10 µs
            
            # Medición de descarga
            GPIO.setup(pin, GPIO.IN)
            inicio = time.time()
            while GPIO.input(pin) and (time.time() - inicio < self.tiempo_lectura):
                pass
            
            duracion = time.time() - inicio
            # Lógica corregida (duración > umbral para línea negra)
            valores.append(duracion > self.umbral)
        return valores

    def calcular_posicion(self, valores):
        """Cálculo de posición con promedio ponderado"""
        if not any(valores):
            return None  # Línea no detectada
            
        total = sum(i * 1000 if val else 0 for i, val in enumerate(valores))  # Corregido
        count = sum(1 for val in valores if val)
        
        return total / count if count != 0 else 3500  # Centro por defecto

    def control_pid(self, posicion):
        """Control PID con anti-windup"""
        if posicion is None:
            return 0
            
        error = posicion - 3500  # Error respecto al centro
        
        # Término proporcional
        P = self.Kp * error
        
        # Término integral
        self.integral += error
        self.integral = max(min(self.integral, 1000), -1000)  # Limitar integral
        I = self.Ki * self.integral
        
        # Término derivativo
        D = self.Kd * (error - self.error_anterior)
        self.error_anterior = error
        
        return P + I + D

    def control_motores(self, izq, der):
        """Control de motores con protección"""
        # Limitar valores
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
        """Algoritmo principal de seguimiento"""
        print("Iniciando seguimiento... (Ctrl+C para detener)")
        try:
            while True:
                valores = self.leer_sensores()
                posicion = self.calcular_posicion(valores)
                
                if posicion is not None:
                    ajuste = self.control_pid(posicion)
                    self.control_motores(self.velocidad_base - ajuste, 
                                       self.velocidad_base + ajuste)
                else:
                    self.control_motores(0, 0)  # Detener si pierde la línea
                
                time.sleep(0.01)
        except KeyboardInterrupt:
            self.detener()

    def prueba_motores(self):
        """Prueba de motores con secuencia"""
        print("\n*** Prueba de motores ***")
        pruebas = [
            ("Izquierdo adelante", 50, 0),
            ("Derecho adelante", 0, 50),
            ("Ambos adelante", 50, 50),
            ("Ambos atrás", -50, -50)
        ]
        
        for nombre, izq, der in pruebas:
            print(nombre)
            self.control_motores(izq, der)
            time.sleep(1.5)
        
        self.detener()
        print("Prueba completada\n")

    def prueba_sensores(self):
        """Monitor en tiempo real de sensores"""
        print("\n*** Monitor de sensores ***")
        try:
            while True:
                valores = [1 if v else 0 for v in self.leer_sensores()]
                print("|".join(f"{v:^3}" for v in valores), end="\r")
                time.sleep(0.1)
        except KeyboardInterrupt:
            print("\nMonitor detenido\n")

    def detener(self):
        """Parada segura"""
        self.control_motores(0, 0)
        self.pwm_a.stop()
        self.pwm_b.stop()

if __name__ == "__main__":
    try:
        robot = RobotSeguidorLinea()
        
        while True:
            print("\n=== MENÚ PRINCIPAL ===")
            print("1. Iniciar seguimiento de línea")
            print("2. Probar motores")
            print("3. Monitor de sensores")
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
                print("Opción no válida.")
    finally:
        GPIO.cleanup()  # Limpieza final al salir
