import jetson_inference
import jetson_utils

# Cargar el modelo de detección de personas
net = jetson_inference.detectNet("pednet", threshold=0.5)

# Fuente de video (cámara CSI o USB)
camera = jetson_utils.videoSource("/dev/video4")  # o "csi://0"
display = jetson_utils.videoOutput()

while display.IsStreaming():
    # Capturar imagen
    img = camera.Capture()

    # Detectar personas
    detections = net.Detect(img)

    # Renderizar con los bounding boxes ya dibujados
    display.Render(img)
    display.SetStatus("Person Detection | Network {:.0f} FPS".format(net.GetNetworkFPS()))

