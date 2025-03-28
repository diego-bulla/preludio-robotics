"""
Módulo de controladores del robot seguidor de línea

Contiene:
- MotorController: Clase para controlar los motores mediante el driver L298N
"""

from .motors import MotorController

__all__ = ['MotorController']
__version__ = '1.0.0'

def get_controller_info():
    """Retorna información de configuración de los controladores"""
    return {
        'motor_pins': MotorController.PINS if hasattr(MotorController, 'PINS') else None,
        'pwm_frequency': MotorController.PWM_FREQ if hasattr(MotorController, 'PWM_FREQ') else None
    }
