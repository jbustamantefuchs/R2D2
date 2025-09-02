import paho.mqtt.client as mqtt
from opcua import Server, ua


# -------------------------------
# Configuración MQTT
# -------------------------------
MQTT_BROKER = "localhost"
MQTT_PORT = 1883
MQTT_CLIENT_ID = "r2d2-opcua"

mqtt_client = mqtt.Client(client_id=MQTT_CLIENT_ID, protocol=mqtt.MQTTv311, transport="tcp")

# -------------------------------
# Configuración OPC UA Server
# -------------------------------
server = Server()
server.set_endpoint("opc.tcp://0.0.0.0:4840/r2d2/server/")
uri = "http://r2d2-opcua.local"
idx = server.register_namespace(uri)

r2d2 = server.nodes.objects.add_object(idx, "R2D2")

# -------------------------------
# Variables OPC UA
# -------------------------------
vars_dict = {
    "InputString": r2d2.add_variable(idx, "InputString", "", ua.VariantType.String),
    "OutputString": r2d2.add_variable(idx, "OutputString", "", ua.VariantType.String),
    "Status": r2d2.add_variable(idx, "Status", "", ua.VariantType.String),
    "ActionString": r2d2.add_variable(idx, "ActionString", "", ua.VariantType.String),
    "MoveString": r2d2.add_variable(idx, "MoveString", "", ua.VariantType.String),
    "Speed": r2d2.add_variable(idx, "Speed", 0, ua.VariantType.Int32),
    "Start": r2d2.add_variable(idx, "Start", False, ua.VariantType.Boolean),
    "Stop": r2d2.add_variable(idx, "Stop", False, ua.VariantType.Boolean),
}

# Hacer todas las variables escribibles
for var in vars_dict.values():
    var.set_writable()

# -------------------------------
# Función para publicar cambios a MQTT
# -------------------------------
def publish_mqtt(var_name, value):
    topic = f"/opcua/{var_name}"
    payload = str(value)
    mqtt_client.publish(topic, payload)
    print(f"[MQTT] {topic} -> {payload}")

# -------------------------------
# Handler de cambios OPC UA
# -------------------------------
class SubHandler:
    def datachange_notification(self, node, val, data):
        var_name = node.get_display_name().Text
        # Evitar publicar OutputString o Status si el cambio viene de MQTT
        if var_name not in ["OutputString", "Status"]:
            publish_mqtt(var_name, val)

    def event_notification(self, event):
        pass

# -------------------------------
# Callback de MQTT
# -------------------------------
def on_message(client, userdata, msg):
    try:
        payload = msg.payload.decode()
        print(f"[MQTT->OPC UA] {msg.topic} = {payload}")

        if msg.topic == "/opcua/OutputString":
            vars_dict["OutputString"].set_value(payload)
        elif msg.topic == "/opcua/Status":
            vars_dict["Status"].set_value(payload)
    except Exception as e:
        print(f"Error en on_message: {e}")

mqtt_client.on_message = on_message
mqtt_client.connect(MQTT_BROKER, MQTT_PORT, 60)
mqtt_client.subscribe("/opcua/OutputString")
mqtt_client.subscribe("/opcua/Status")

# -------------------------------
# Iniciar servidor OPC UA
# -------------------------------
server.start()
print("Servidor OPC UA corriendo en opc.tcp://0.0.0.0:4840/r2d2/server/")

# Crear suscripción OPC UA
handler = SubHandler()
sub = server.create_subscription(500, handler)
for var in vars_dict.values():
    sub.subscribe_data_change(var)

# -------------------------------
# Loop principal
# -------------------------------
try:
    while True:
        mqtt_client.loop(timeout=1.0)
except KeyboardInterrupt:
    print("Apagando servidor...")
    server.stop()
    mqtt_client.disconnect()

