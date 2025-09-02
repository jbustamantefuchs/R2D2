import paho.mqtt.client as mqtt
import rospy
from std_msgs.msg import String
import signal
import sys

# Inicializar el nodo ROS
rospy.init_node("mqtt_to_ros", anonymous=True)
pub = rospy.Publisher("nr_controller", String, queue_size=10)

# Diccionario para almacenar los valores de direction y speed
datos = {
    "direction": None,
    "speed": None
}

# Variable para almacenar el ?ltimo estado enviado
ultimo_estado = None

# Callback cuando se conecta al broker
def on_connect(client, userdata, flags, rc):
    print("Conectado con resultado: " + str(rc))
    client.subscribe("direction")
    client.subscribe("speed")

# Callback cuando recibe un mensaje
def on_message(client, userdata, msg):
    global ultimo_estado

    if msg.topic in datos:
        nuevo_valor = msg.payload.decode() or "0"
        if datos[msg.topic] != nuevo_valor:
            datos[msg.topic] = nuevo_valor
            print(f"Mensaje actualizado en el tema {msg.topic}: {datos[msg.topic]}")
        else:
            return  # No hacer nada si el valor no cambi?

    estado_actual = (
        datos["direction"] if datos["direction"] is not None else "0",
        datos["speed"] if datos["speed"] is not None else "0"
    )

    if estado_actual != ultimo_estado:
        mensaje_ros = f"({estado_actual[0]},{estado_actual[1]})"
        pub.publish(mensaje_ros)
        print(f"Publicado en ROS: {mensaje_ros}")
        ultimo_estado = estado_actual

# Crear cliente MQTT
client = mqtt.Client()
client.on_connect = on_connect
client.on_message = on_message
client.connect("localhost", 1883, 60)

# Manejar Ctrl+C para salir limpiamente
def signal_handler(sig, frame):
    print("\nCerrando MQTT y ROS...")
    client.disconnect()
    rospy.signal_shutdown("Cierre por Ctrl+C")
    sys.exit(0)

signal.signal(signal.SIGINT, signal_handler)

# Iniciar el bucle MQTT
try:
    client.loop_forever()
except KeyboardInterrupt:
    signal_handler(None, None)
