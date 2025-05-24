import serial
import time
from typing import Tuple, Optional

def comunicacion_serial_arduino(
    baudrate: int = 9600,
    tiempo_espera: float = 1.0,
    senial_envio: str = 'I',
    num_respuestas: int = 1,
    timeout: float = 5.0
) -> Tuple[Optional[str], ...]:
    """
    Envía una señal a Arduino a través de los pines TX/RX (UART) y espera respuestas.

    Args:
        baudrate (int): Velocidad de transmisión en baudios (por defecto 9600).
        tiempo_espera (float): Tiempo de espera después de enviar la señal (por defecto 1.0 segundos).
        senial_envio (str): Señal a enviar al Arduino (por defecto 'INICIAR').
        num_respuestas (int): Número de respuestas esperadas desde el Arduino (por defecto 1).
        timeout (float): Tiempo máximo de espera para recibir cada respuesta (por defecto 5.0 segundos).

    Returns:
        Tuple[Optional[str], ...]: Tupla con las respuestas recibidas. Si hay error, retorna tupla con None.

    Raises:
        serial.SerialException: Si hay un error con el puerto serial.
    """
    try:
        # Configurar el puerto serial UART (ttyS0 en Raspberry Pi 4)
        with serial.Serial(
            port='/dev/ttyS0',  # Puerto UART hardware (GPIO TX/RX)
            baudrate=baudrate,
            timeout=timeout,
            bytesize=serial.EIGHTBITS,
            parity=serial.PARITY_NONE,
            stopbits=serial.STOPBITS_ONE
        ) as ser:
            # Limpiar buffers para evitar datos residuales
            ser.reset_input_buffer()
            ser.reset_output_buffer()

            # Enviar la señal al Arduino
            ser.write(f"{senial_envio}\n".encode('utf-8'))
            
            # Esperar un momento para que el Arduino procese la señal
            ser.flush()
            time.sleep(tiempo_espera)

            respuestas = []
            for _ in range(num_respuestas):
                # Leer una línea desde el puerto serial
                linea = ser.readline().decode('utf-8').strip()
                if linea:
                    respuestas.append(linea)
                else:
                    respuestas.append(None)  # Timeout o no hay datos

            return tuple(respuestas)

    except serial.SerialException as e:
        print(f"Error de comunicación serial: {e}")
        return tuple([None] * num_respuestas)
