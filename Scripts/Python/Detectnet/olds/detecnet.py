#!/usr/bin/env python3
import rospy
from std_msgs.msg import Int32
import jetson_inference
import jetson_utils
import time

# Inicializa ROS
rospy.init_node('detector_personas_node')
pub = rospy.Publisher('/persona_detectada', Int32, queue_size=10)

# Carga la red de detección
net = jetson_inference.detectNet("ssd-mobilenet-v2", threshold=0.5)
camera = jetson_utils.videoSource("/dev/video4")  # Cámara USB
display = jetson_utils.videoOutput()

rospy.loginfo("Detector de personas iniciado")

estado_anterior = 0  # Estado inicial: sin persona
esperando_confirmacion = False
tiempo_cambio = 0
estado_pendiente = None

# Bucle principal
while not rospy.is_shutdown():
    img = camera.Capture()
    detecciones = net.Detect(img)
    display.Render(img)
    display.SetStatus("Detección de personas | Jose")

    # Verifica si hay personas
    personas = [d for d in detecciones if net.GetClassDesc(d.ClassID) == "person"]
    estado_actual = 1 if personas else 0

    # Si no estamos esperando confirmación y hay cambio de estado
    if not esperando_confirmacion and estado_actual != estado_anterior:
        esperando_confirmacion = True
        tiempo_cambio = time.time()
        estado_pendiente = estado_actual
        rospy.loginfo(f"Cambio detectado ? Esperando confirmación de {estado_pendiente}...")

    # Si estamos esperando confirmación
    if esperando_confirmacion:
        # Han pasado 3 segundos
        if time.time() - tiempo_cambio >= 3.0:
            # Capturamos nueva imagen para confirmar estado
            img_confirm = camera.Capture()
            detecciones_confirm = net.Detect(img_confirm)
            personas_confirm = [d for d in detecciones_confirm if net.GetClassDesc(d.ClassID) == "person"]
            estado_confirmado = 1 if personas_confirm else 0

            # Solo publica si el estado se mantiene
            if estado_confirmado == estado_pendiente:
                rospy.loginfo(f"Estado confirmado: {estado_confirmado}")
                pub.publish(estado_confirmado)
                estado_anterior = estado_confirmado
            else:
                rospy.loginfo("Cambio cancelado, el estado ya no es el mismo.")

            # Reinicia variables
            esperando_confirmacion = False
            estado_pendiente = None


