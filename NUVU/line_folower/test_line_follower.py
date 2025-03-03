import RPi.GPIO as GPIO
import time

class RobotSeguidorLinea:
    def __init__(self):
        # Configuración inicial GPIO
        GPIO.setmode(GPIO.BCM)
        GPIO.setwarnings(False)
        
        # Pines para QTR-8RC
        self.SENSOR_PINS = [24, 25, 8, 7, 12, 16, 20, 21]
        
        # Pines para L298N
        self.ENA = 26  # PWM motor A
        self.IN1 = 19  # Control dirección motor A
        self.IN2 = 13
        self.ENB = 11  # PWM motor B
        self.IN3 = 6  # Control dirección motor B
        self.IN4 = 5
        
        # Configurar todos los pines
        self.setup_pins()
        
        # Variables de control
        self.velocidad_base = 50  # Velocidad base (0-100)
        self.tiempo_lectura = 0.1  # Tiempo para lectura de sensores en ms
        self.umbral = 0.5  # Umbral para detectar línea negra
        
        # Variables PID
        self.Kp = 0.5
        self.Ki = 0.0001
        self.Kd = 0.1
        self.error_anterior = 0
        self.integral = 0
        
    def setup_pins(self):
        """Configura todos los pines GPIO"""
        # Configurar pines de motores
        motor_pins = [self.ENA, self.IN1, self.IN2, self.ENB, self.IN3, self.IN4]
        for pin in motor_pins:
            GPIO.setup(pin, GPIO.OUT)
            
        # Crear objetos PWM
        self.pwm_a = GPIO.PWM(self.ENA, 1000)  # 1000 Hz
        self.pwm_b = GPIO.PWM(self.ENB, 1000)
        self.pwm_a.start(0)
        self.pwm_b.start(0)
        
        # Configurar pines de sensores como entrada
        for pin in self.SENSOR_PINS:
            GPIO.setup(pin, GPIO.IN)
    
    def leer_sensores(self):
        """Lee los valores del QTR-8RC usando RC timing"""
        valores = []
        
        for pin in self.SENSOR_PINS:
            # Cargar el capacitor
            GPIO.setup(pin, GPIO.OUT)
            GPIO.output(pin, GPIO.HIGH)
            time.sleep(0.00001)  # 10 microsegundos de carga
            
            # Cambiar a modo lectura
            GPIO.setup(pin, GPIO.IN)
            
            # Medir tiempo de descarga
            inicio = time.time()
            duracion = 0
            
            # Esperar hasta que el pin se descargue o timeout
            while GPIO.input(pin) and duracion < self.tiempo_lectura:
                duracion = time.time() - inicio
            
            valores.append(duracion < self.umbral)  # True para línea negra
            
        return valores
    
    def calcular_posicion(self, valores_sensores):
        """Calcula la posición de la línea"""
        if not any(valores_sensores):  # Si no se detecta línea
            return None
            
        numerador = 0
        denominador = 0
        
        for i, valor in enumerate(valores_sensores):
            if valor:
                numerador += i * 1000
                denominador += 1
                
        if denominador == 0:
            return 3500  # Posición central
            
        return numerador / denominador
    
    def control_pid(self, posicion):
        """Implementa el control PID"""
        if posicion is None:
            return 0
            
        error = posicion - 3500  # Error respecto al centro
        
        # Componente proporcional
        p = self.Kp * error
        
        # Componente integral
        self.integral += error
        self.integral = max(min(self.integral, 1000), -1000)  # Anti-windup
        i = self.Ki * self.integral
        
        # Componente derivativa
        d = self.Kd * (error - self.error_anterior)
        self.error_anterior = error
        
        return p + i + d
    
    def control_motores(self, velocidad_izq, velocidad_der):
        """Controla los motores con límites de seguridad"""
        # Limitar velocidades entre -100 y 100
        velocidad_izq = max(min(velocidad_izq, 100), -100)
        velocidad_der = max(min(velocidad_der, 100), -100)
        
        # Motor izquierdo
        if velocidad_izq >= 0:
            GPIO.output(self.IN1, GPIO.HIGH)
            GPIO.output(self.IN2, GPIO.LOW)
        else:
            GPIO.output(self.IN1, GPIO.LOW)
            GPIO.output(self.IN2, GPIO.HIGH)
        self.pwm_a.ChangeDutyCycle(abs(velocidad_izq))
        
        # Motor derecho
        if velocidad_der >= 0:
            GPIO.output(self.IN3, GPIO.HIGH)
            GPIO.output(self.IN4, GPIO.LOW)
        else:
            GPIO.output(self.IN3, GPIO.LOW)
            GPIO.output(self.IN4, GPIO.HIGH)
        self.pwm_b.ChangeDutyCycle(abs(velocidad_der))
    
    def seguir_linea(self):
        """Función principal de seguimiento de línea"""
        print("Iniciando seguimiento de línea... Presiona Ctrl+C para detener")
        
        try:
            while True:
                # Leer sensores
                valores = self.leer_sensores()
                
                # Calcular posición
                posicion = self.calcular_posicion(valores)
                
                if posicion is not None:
                    # Calcular corrección PID
                    correccion = self.control_pid(posicion)
                    
                    # Calcular velocidades de motores
                    velocidad_izq = self.velocidad_base - correccion
                    velocidad_der = self.velocidad_base + correccion
                    
                    # Aplicar a motores
                    self.control_motores(velocidad_izq, velocidad_der)
                else:
                    # Si se pierde la línea, detenerse
                    self.control_motores(0, 0)
                
                time.sleep(0.01)  # Pequeña pausa
                
        except KeyboardInterrupt:
            self.detener()
            print("\nPrograma detenido por el usuario")
    
    def prueba_motores(self):
        """Realiza una prueba básica de los motores"""
        print("Probando motores...")
        
        # Probar motor izquierdo
        print("Motor izquierdo adelante")
        self.control_motores(50, 0)
        time.sleep(2)
        
        # Probar motor derecho
        print("Motor derecho adelante")
        self.control_motores(0, 50)
        time.sleep(2)
        
        # Ambos motores
        print("Ambos motores adelante")
        self.control_motores(50, 50)
        time.sleep(2)
        
        self.detener()
        print("Prueba de motores completada")
    
    def prueba_sensores(self):
        """Realiza una prueba de lectura de sensores"""
        print("Probando sensores... Presiona Ctrl+C para detener")
        try:
            while True:
                valores = self.leer_sensores()
                print("Valores sensores:", valores)
                time.sleep(0.5)
        except KeyboardInterrupt:
            print("\nPrueba de sensores detenida")
    
    def detener(self):
        """Detiene los motores y limpia GPIO"""
        self.control_motores(0, 0)
        self.pwm_a.stop()
        self.pwm_b.stop()
        GPIO.cleanup()

if __name__ == "__main__":
    robot = RobotSeguidorLinea()
    
    while True:
        print("\nMenú del Robot Seguidor de Línea:")
        print("1. Iniciar seguimiento de línea")
        print("2. Probar motores")
        print("3. Probar sensores")
        print("4. Salir")
        
        opcion = input("Selecciona una opción: ")
        
        if opcion == "1":
            robot.seguir_linea()
        elif opcion == "2":
            robot.prueba_motores()
        elif opcion == "3":
            robot.prueba_sensores()
        elif opcion == "4":
            robot.detener()
            break
        else:
            print("Opción no válida")
