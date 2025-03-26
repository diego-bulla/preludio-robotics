import cv2

class Camera:
    def __init__(self, source=0):
        """Inicializa la cámara. 'source' puede ser 0 (webcam) o una dirección de PixyCam."""
        self.cap = cv2.VideoCapture(source)
        if not self.cap.isOpened():
            raise Exception("Error: No se pudo acceder a la cámara.")

    def get_frame(self):
        """Captura un frame de la cámara y lo devuelve en formato OpenCV (numpy array)."""
        ret, frame = self.cap.read()
        if not ret:
            raise Exception("Error: No se pudo capturar el frame.")
        return frame

    def release(self):
        """Libera la cámara."""
        self.cap.release()
        cv2.destroyAllWindows()

if __name__ == "__main__":
    cam = Camera()
    try:
        while True:
            frame = cam.get_frame()
            cv2.imshow("Camera Feed", frame)
            if cv2.waitKey(1) & 0xFF == ord('q'):
                break
    except KeyboardInterrupt:
        pass
    finally:
        cam.release()
