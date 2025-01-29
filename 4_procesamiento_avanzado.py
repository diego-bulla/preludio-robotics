# %%
#!/home/nuvu-pc-y764/Documents/NUVU/PRELUDIO/python_venv/bin/python
# -*- coding: utf-8 -*-

"""
PROCESAMIENTO AVANZADO

@author: nuvu-pc-y764
"""

# %% Deteccion de contornos basicos

import cv2
import numpy as np
def detectar_contornos(numero_camara=0):
    """
    Demuestra la detección básica de contornos en tiempo real.
    Muestra la imagen original y los contornos detectados.
    Se puede cerrar presionando 'q' en cualquier ventana.
    Args:
        numero_camara: Índice de la cámara a utilizar
    """
    camara = cv2.VideoCapture(numero_camara)
    if not camara.isOpened():
        print("Error: No se pudo acceder a la cámara")
        return
    # Crear ventanas
    cv2.namedWindow('Original', cv2.WINDOW_NORMAL)
    cv2.namedWindow('Contornos', cv2.WINDOW_NORMAL)
    try:
        while True:
            ret, frame = camara.read()
            if ret:
                # Convertir a escala de grises
                gris = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
                # Aplicar blur para reducir ruido
                blur = cv2.GaussianBlur(gris, (5, 5), 0)
                # Detectar bordes con Canny
                bordes = cv2.Canny(blur, 50, 150)
                # Encontrar contornos
                contornos, _ = cv2.findContours(bordes, cv2.RETR_EXTERNAL,
                                              cv2.CHAIN_APPROX_SIMPLE)
                # Dibujar contornos
                imagen_contornos = frame.copy()
                cv2.drawContours(imagen_contornos, contornos, -1, (0, 0, 255), 2)
                # Mostrar número de contornos detectados
                cv2.putText(imagen_contornos, f'Contornos: {len(contornos)}',
                          (10, 30), cv2.FONT_HERSHEY_SIMPLEX, 1,
                          (255, 0, 0), 2)
                # Mostrar las imágenes
                cv2.imshow('Original', frame)
                cv2.imshow('Contornos', imagen_contornos)
                # Salir si se presiona 'q'
                if cv2.waitKey(1) & 0xFF == ord('q'):
                    break
            else:
                print("Error en la captura del frame")
                break
    finally:
        # Liberar recursos
        camara.release()
        cv2.destroyAllWindows()
        print("Detección de contornos finalizada")
# Ejemplo de uso
detectar_contornos(0)

# %% Deteccion de contornos avanzado con centroides

import cv2
import numpy as np
def detectar_formas(numero_camara=0):
    """
    Detecta formas geométricas básicas y sus centroides en tiempo real.
    Muestra la imagen original y las formas detectadas con sus centroides.
    Se puede cerrar presionando 'q' en cualquier ventana.
    Args:
        numero_camara: Índice de la cámara a utilizar
    """
    camara = cv2.VideoCapture(numero_camara)
    if not camara.isOpened():
        print("Error: No se pudo acceder a la cámara")
        return
    # Crear ventanas
    cv2.namedWindow('Original', cv2.WINDOW_NORMAL)
    cv2.namedWindow('Formas Detectadas', cv2.WINDOW_NORMAL)
    def detectar_forma(contorno):
        """Detecta el tipo de forma basado en el número de vértices."""
        perimetro = cv2.arcLength(contorno, True)
        approx = cv2.approxPolyDP(contorno, 0.04 * perimetro, True)
        if len(approx) == 3:
            return "Triangulo"
        elif len(approx) == 4:
            return "Rectangulo"
        elif len(approx) >= 8:
            return "Circulo"
        return "Desconocido"
    try:
        while True:
            ret, frame = camara.read()
            if ret:
                # Convertir a escala de grises
                gris = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
                blur = cv2.GaussianBlur(gris, (5, 5), 0)
                _, umbral = cv2.threshold(blur, 60, 255, cv2.THRESH_BINARY)
                # Encontrar contornos
                contornos, _ = cv2.findContours(umbral, cv2.RETR_EXTERNAL,
                                              cv2.CHAIN_APPROX_SIMPLE)
                # Copiar frame para dibujar
                imagen_formas = frame.copy()
                # Procesar cada contorno
                for contorno in contornos:
                    # Filtrar por área mínima
                    if cv2.contourArea(contorno) < 500:
                        continue
                    # Calcular centroide
                    M = cv2.moments(contorno)
                    if M["m00"] != 0:
                        cx = int(M["m10"] / M["m00"])
                        cy = int(M["m01"] / M["m00"])
                        # Detectar forma
                        forma = detectar_forma(contorno)
                        # Dibujar contorno y centroide
                        cv2.drawContours(imagen_formas, [contorno], -1,
                                       (0, 255, 0), 2)
                        cv2.circle(imagen_formas, (cx, cy), 7, (0, 0, 255), -1)
                        cv2.putText(imagen_formas, forma, (cx - 20, cy - 20),
                                  cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255, 0, 0), 2)
                # Mostrar las imágenes
                cv2.imshow('Original', frame)
                cv2.imshow('Formas Detectadas', imagen_formas)
                # Salir si se presiona 'q'
                if cv2.waitKey(1) & 0xFF == ord('q'):
                    break
            else:
                print("Error en la captura del frame")
                break
    finally:
        # Liberar recursos
        camara.release()
        cv2.destroyAllWindows()
        print("Detección de formas finalizada")
# Ejemplo de uso
detectar_formas(0)

# %% Tracking de objetos

import cv2
import numpy as np
def tracking_objetos(numero_camara=0):
    """
    Realiza tracking de objetos por color, calcula distancias y posiciones.
    Permite ajustar los rangos HSV con trackbars.
    Se puede cerrar presionando 'q' en cualquier ventana.
    Args:
        numero_camara: Índice de la cámara a utilizar
    """
    camara = cv2.VideoCapture(numero_camara)
    if not camara.isOpened():
        print("Error: No se pudo acceder a la cámara")
        return
    # Crear ventanas
    cv2.namedWindow('Controles', cv2.WINDOW_NORMAL)
    cv2.namedWindow('Original', cv2.WINDOW_NORMAL)
    cv2.namedWindow('Mascara', cv2.WINDOW_NORMAL)
    cv2.namedWindow('Tracking', cv2.WINDOW_NORMAL)
    # Crear trackbars para ajustar HSV
    def nada(x):
        pass
    # Valores iniciales para detectar color rojo
    cv2.createTrackbar('H min', 'Controles', 0, 179, nada)
    cv2.createTrackbar('H max', 'Controles', 10, 179, nada)
    cv2.createTrackbar('S min', 'Controles', 100, 255, nada)
    cv2.createTrackbar('S max', 'Controles', 255, 255, nada)
    cv2.createTrackbar('V min', 'Controles', 100, 255, nada)
    cv2.createTrackbar('V max', 'Controles', 255, 255, nada)
    # Punto de referencia para medir distancias (centro de la imagen)
    _, frame = camara.read()
    altura, ancho = frame.shape[:2]
    punto_ref = (ancho // 2, altura // 2)
    try:
        while True:
            ret, frame = camara.read()
            if ret:
                # Convertir a HSV
                hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)
                # Obtener valores de trackbars
                h_min = cv2.getTrackbarPos('H min', 'Controles')
                h_max = cv2.getTrackbarPos('H max', 'Controles')
                s_min = cv2.getTrackbarPos('S min', 'Controles')
                s_max = cv2.getTrackbarPos('S max', 'Controles')
                v_min = cv2.getTrackbarPos('V min', 'Controles')
                v_max = cv2.getTrackbarPos('V max', 'Controles')
                # Crear máscara
                rango_bajo = np.array([h_min, s_min, v_min])
                rango_alto = np.array([h_max, s_max, v_max])
                mascara = cv2.inRange(hsv, rango_bajo, rango_alto)
                # Aplicar operaciones morfológicas
                kernel = np.ones((5,5), np.uint8)
                mascara = cv2.erode(mascara, kernel, iterations=1)
                mascara = cv2.dilate(mascara, kernel, iterations=2)
                # Encontrar contornos
                contornos, _ = cv2.findContours(mascara, cv2.RETR_EXTERNAL,
                                              cv2.CHAIN_APPROX_SIMPLE)
                # Imagen para tracking
                tracking = frame.copy()
                # Dibujar punto de referencia
                cv2.circle(tracking, punto_ref, 5, (0, 0, 255), -1)
                cv2.putText(tracking, "Referencia",
                          (punto_ref[0] + 10, punto_ref[1]),
                          cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 0, 255), 2)
                # Procesar objetos detectados
                for contorno in contornos:
                    area = cv2.contourArea(contorno)
                    if area > 500:  # Filtrar objetos pequeños
                        # Calcular centro del objeto
                        M = cv2.moments(contorno)
                        if M["m00"] != 0:
                            cx = int(M["m10"] / M["m00"])
                            cy = int(M["m01"] / M["m00"])
                            # Calcular distancia al punto de referencia
                            distancia = np.sqrt((cx - punto_ref[0])**2 +
                                              (cy - punto_ref[1])**2)
                            # Dibujar contorno y centro
                            cv2.drawContours(tracking, [contorno], -1,
                                           (0, 255, 0), 2)
                            cv2.circle(tracking, (cx, cy), 7, (255, 0, 0), -1)
                            # Dibujar línea al punto de referencia
                            cv2.line(tracking, (cx, cy), punto_ref,
                                   (255, 0, 0), 2)
                            # Mostrar distancia
                            cv2.putText(tracking, f"Dist: {int(distancia)}px",
                                      (cx + 10, cy), cv2.FONT_HERSHEY_SIMPLEX,
                                      0.7, (255, 0, 0), 2)
                # Mostrar imágenes
                cv2.imshow('Original', frame)
                cv2.imshow('Mascara', mascara)
                cv2.imshow('Tracking', tracking)
                # Salir si se presiona 'q'
                if cv2.waitKey(1) & 0xFF == ord('q'):
                    break
            else:
                print("Error en la captura del frame")
                break
    finally:
        # Liberar recursos
        camara.release()
        cv2.destroyAllWindows()
        print("Tracking de objetos finalizado")
# Ejemplo de uso
# tracking_objetos(0)import cv2
import numpy as np
def tracking_objetos(numero_camara=0):
    """
    Realiza tracking de objetos por color, calcula distancias y posiciones.
    Permite ajustar los rangos HSV con trackbars.
    Se puede cerrar presionando 'q' en cualquier ventana.
    Args:
        numero_camara: Índice de la cámara a utilizar
    """
    camara = cv2.VideoCapture(numero_camara)
    if not camara.isOpened():
        print("Error: No se pudo acceder a la cámara")
        return
    # Crear ventanas
    cv2.namedWindow('Controles', cv2.WINDOW_NORMAL)
    cv2.namedWindow('Original', cv2.WINDOW_NORMAL)
    cv2.namedWindow('Mascara', cv2.WINDOW_NORMAL)
    cv2.namedWindow('Tracking', cv2.WINDOW_NORMAL)
    # Crear trackbars para ajustar HSV
    def nada(x):
        pass
    # Valores iniciales para detectar color rojo
    cv2.createTrackbar('H min', 'Controles', 0, 179, nada)
    cv2.createTrackbar('H max', 'Controles', 10, 179, nada)
    cv2.createTrackbar('S min', 'Controles', 100, 255, nada)
    cv2.createTrackbar('S max', 'Controles', 255, 255, nada)
    cv2.createTrackbar('V min', 'Controles', 100, 255, nada)
    cv2.createTrackbar('V max', 'Controles', 255, 255, nada)
    # Punto de referencia para medir distancias (centro de la imagen)
    _, frame = camara.read()
    altura, ancho = frame.shape[:2]
    punto_ref = (ancho // 2, altura // 2)
    try:
        while True:
            ret, frame = camara.read()
            if ret:
                # Convertir a HSV
                hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)
                # Obtener valores de trackbars
                h_min = cv2.getTrackbarPos('H min', 'Controles')
                h_max = cv2.getTrackbarPos('H max', 'Controles')
                s_min = cv2.getTrackbarPos('S min', 'Controles')
                s_max = cv2.getTrackbarPos('S max', 'Controles')
                v_min = cv2.getTrackbarPos('V min', 'Controles')
                v_max = cv2.getTrackbarPos('V max', 'Controles')
                # Crear máscara
                rango_bajo = np.array([h_min, s_min, v_min])
                rango_alto = np.array([h_max, s_max, v_max])
                mascara = cv2.inRange(hsv, rango_bajo, rango_alto)
                # Aplicar operaciones morfológicas
                kernel = np.ones((5,5), np.uint8)
                mascara = cv2.erode(mascara, kernel, iterations=1)
                mascara = cv2.dilate(mascara, kernel, iterations=2)
                # Encontrar contornos
                contornos, _ = cv2.findContours(mascara, cv2.RETR_EXTERNAL,
                                              cv2.CHAIN_APPROX_SIMPLE)
                # Imagen para tracking
                tracking = frame.copy()
                # Dibujar punto de referencia
                cv2.circle(tracking, punto_ref, 5, (0, 0, 255), -1)
                cv2.putText(tracking, "Referencia",
                          (punto_ref[0] + 10, punto_ref[1]),
                          cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 0, 255), 2)
                # Procesar objetos detectados
                for contorno in contornos:
                    area = cv2.contourArea(contorno)
                    if area > 500:  # Filtrar objetos pequeños
                        # Calcular centro del objeto
                        M = cv2.moments(contorno)
                        if M["m00"] != 0:
                            cx = int(M["m10"] / M["m00"])
                            cy = int(M["m01"] / M["m00"])
                            # Calcular distancia al punto de referencia
                            distancia = np.sqrt((cx - punto_ref[0])**2 +
                                              (cy - punto_ref[1])**2)
                            # Dibujar contorno y centro
                            cv2.drawContours(tracking, [contorno], -1,
                                           (0, 255, 0), 2)
                            cv2.circle(tracking, (cx, cy), 7, (255, 0, 0), -1)
                            # Dibujar línea al punto de referencia
                            cv2.line(tracking, (cx, cy), punto_ref,
                                   (255, 0, 0), 2)
                            # Mostrar distancia
                            cv2.putText(tracking, f"Dist: {int(distancia)}px",
                                      (cx + 10, cy), cv2.FONT_HERSHEY_SIMPLEX,
                                      0.7, (255, 0, 0), 2)
                # Mostrar imágenes
                cv2.imshow('Original', frame)
                cv2.imshow('Mascara', mascara)
                cv2.imshow('Tracking', tracking)
                # Salir si se presiona 'q'
                if cv2.waitKey(1) & 0xFF == ord('q'):
                    break
            else:
                print("Error en la captura del frame")
                break
    finally:
        # Liberar recursos
        camara.release()
        cv2.destroyAllWindows()
        print("Tracking de objetos finalizado")
# Ejemplo de uso
tracking_objetos(0)

# %% Tarea

"""
Los estudiantes deberán identificar cuales son los valores ideales de HSV
para ambos colores de fichas
"""