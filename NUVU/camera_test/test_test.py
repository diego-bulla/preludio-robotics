from picamera2 import Picamera2

# Inicializar la cámara
picam2 = Picamera2()

# Configurar la cámara para modo preview
config = picam2.create_preview_configuration()
picam2.configure(config)

# Iniciar la cámara
picam2.start()

# Capturar una imagen
picam2.capture_file("test_image.jpg")
print("Imagen capturada y guardada como 'test_image.jpg'")

# Detener la cámara
picam2.stop()
