"""
Módulo de sensores del robot seguidor de línea

Contiene:
- LineSensor: Clase para lectura del array de sensores QTR-8RC
"""

from .line_sensor import LineSensor

__all__ = ['LineSensor']
__version__ = '1.0.0'

def get_sensor_config():
    """Retorna configuración básica de los sensores"""
    return {
        'sensor_count': len(LineSensor.PINS) if hasattr(LineSensor, 'PINS') else 6,
        'sensor_type': 'QTR-8RC (Modo RC)'
    }
