from sensor_reader import comunicacion_serial_arduino 

if __name__ == "__main__":
    try:
        respuestas = comunicacion_serial_arduino(
            senial_envio="I",
            num_respuestas=3
        )
        print(f"Respuestas recibidas: {respuestas}")
    except Exception as e:
        print(f"Error: {e}")
