import time
from controllers.motors import MotorController
from controllers.leds import LEDController
from controllers.servos import ServoController
from sensors.line_sensor import LineSensor
from sensors.object_detector import ObjectDetector
import config

def main():
    print("Iniciando robot...")

    # Inicialización de controladores
    motors = MotorController(config.MOTOR_PINS)
    leds = LEDController(config.LED_PINS)
    servos = ServoController(config.SERVO_PINS)
    line_sensor = LineSensor(config.LINE_THRESHOLD)
    object_detector = ObjectDetector()

    try:
        while True:
            # Lógica de seguimiento de línea
            if line_sensor.detect_line():
                motors.forward()
                leds.indicate("avanzar")
            else:
                motors.stop()
                leds.indicate("parar")

            # Detección de objetos
            obj_position = object_detector.detect()
            if obj_position:
                motors.move_towards(obj_position)
                servos.push_object()

            time.sleep(0.1)

    except KeyboardInterrupt:
        print("Deteniendo robot...")
        motors.stop()
        leds.indicate("parar")

if __name__ == "__main__":
    main()
