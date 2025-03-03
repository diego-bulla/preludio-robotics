from picamera2 import Picamera2
picam = Picamera2()
config = picam.create_still_configuration()
picam.configure(config)
picam.start()
picam.capture_file("foto.jpg")
picam.close()
