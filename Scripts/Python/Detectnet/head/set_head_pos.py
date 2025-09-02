import jetson_inference
import jetson_utils
import cv2
import numpy as np
import requests
import serial
import time

# -------------------------
# Configuración DetectNet
# -------------------------
net = jetson_inference.detectNet("ssd-mobilenet-v2", threshold=0.5)

# -------------------------
# Configuración Serial
# -------------------------
ser = serial.Serial('/dev/ttyACM1', 9600, timeout=1)  # Ajusta el puerto si es distinto
time.sleep(2)  # Espera que arranque el Teensy

# Stream MJPEG
url = "http://127.0.0.1:5000/video_feed"
stream = requests.get(url, stream=True)
bytes_data = b''

# -------------------------
# Configuración display
# -------------------------
DISPLAY_WIDTH = 320  # pequeño para VNC seguro
DISPLAY_HEIGHT = 240

# -------------------------
# Loop principal
# -------------------------
for chunk in stream.iter_content(chunk_size=1024):
    bytes_data += chunk
    a = bytes_data.find(b'\xff\xd8')
    b = bytes_data.find(b'\xff\xd9')
    if a != -1 and b != -1:
        jpg = bytes_data[a:b+2]
        bytes_data = bytes_data[b+2:]
        frame = cv2.imdecode(np.frombuffer(jpg, dtype=np.uint8), cv2.IMREAD_COLOR)
        if frame is not None:
            # Redimensionar para display seguro
            frame_small = cv2.resize(frame, (DISPLAY_WIDTH, DISPLAY_HEIGHT))
            height, width, _ = frame_small.shape
            third_width = width // 3

            # Dibujar líneas divisorias
            cv2.line(frame_small, (third_width, 0), (third_width, height), (0, 255, 0), 1)
            cv2.line(frame_small, (2*third_width, 0), (2*third_width, height), (0, 255, 0), 1)

            # Convertir a RGBA y enviar a GPU para DetectNet
            frame_rgba = cv2.cvtColor(frame_small, cv2.COLOR_BGR2RGBA)
            cuda_frame = jetson_utils.cudaFromNumpy(frame_rgba)

            # Detectar personas
            detections = net.Detect(cuda_frame)
            for det in detections:
                if det.ClassID == 1:  # solo personas
                    x_center = (det.Left + det.Right) / 2
                    if x_center < third_width:
                        section = "Izquierda"
                        ser.write(b'L')
                    elif x_center < 2*third_width:
                        section = "Centro"
                        ser.write(b'C')
                    else:
                        section = "Derecha"
                        ser.write(b'R')

                    print(f"Persona detectada en: {section}")

                    # Dibujar bounding box sobre frame
                    cv2.rectangle(frame_small,
                                  (int(det.Left), int(det.Top)),
                                  (int(det.Right), int(det.Bottom)),
                                  (0, 0, 255), 1)

                    break  # solo procesar la primera persona

            # -------------------------
            # Mostrar frame seguro en VNC
            # -------------------------
            cv2.imshow("Video Seguro", frame_small)
            if cv2.waitKey(1) & 0xFF == ord('q'):
                break

cv2.destroyAllWindows()
ser.close()

