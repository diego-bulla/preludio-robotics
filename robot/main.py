# main.py

import time
import logging
from motors import MotorController

logging.basicConfig(level=logging.INFO)

def probar_movimientos():
    motor = MotorController()
    
    try:
        print("Avanzando...")
        motor.avanzar(60)
        time.sleep(2)

        print("Deteniéndose...")
        motor.control_motores(0, 0)
        time.sleep(1)

        print("Retrocediendo...")
        motor.retroceder(60)
        time.sleep(2)

        print("Deteniéndose...")
        motor.control_motores(0, 0)
        time.sleep(1)

        print("Girando a la izquierda...")
        motor.girar_izquierda(50)
        time.sleep(1.5)

        print("Deteniéndose...")
        motor.control_motores(0, 0)
        time.sleep(1)

        print("Girando a la derecha...")
        motor.girar_derecha(50)
        time.sleep(1.5)

        print("Deteniéndose...")
        motor.control_motores(0, 0)

    except KeyboardInterrupt:
        print("\nInterrumpido por el usuario.")
    finally:
        print("Limpieza de GPIO...")
        motor.detener()

if __name__ == "__main__":
    probar_movimientos()
