"""
Módulo de utilidades para gestión de hardware y seguridad

Componentes principales:
1. GPIOManager: Control centralizado de recursos GPIO
   - Métodos: init(), reserve_pin(), release_pin(), cleanup()
2. gpio_protected: Decorador para operaciones seguras con GPIO
3. SafetyManager: Monitorización de temperatura/voltaje (placeholder)

Uso típico:
>>> from utils import GPIOManager
>>> GPIOManager.init()
>>> GPIOManager.reserve_pin(18)
"""

import logging
from .gpio_manager import GPIOManager, gpio_protected
from .safety_monitor import SafetyManager  # Futura implementación

__all__ = [
    'GPIOManager',
    'gpio_protected',
    'SafetyManager'
]

__version__ = '2.1.0'

# Configura logging para el módulo
logging.basicConfig(
    level=logging.INFO,
    format='%(asctime)s - [UTILS] - %(levelname)s - %(message)s'
)

def check_dependencies():
    """Verifica dependencias críticas del sistema"""
    try:
        import RPi.GPIO as GPIO
        return True
    except ImportError:
        logging.error("Paquete RPi.GPIO no encontrado - Solo modo simulación")
        return False

# Inicialización segura al importar
if check_dependencies():
    GPIOManager.init(cleanup_atexit=True)
else:
    logging.warning("Modo simulación activado - Sin acceso a GPIO físico")
