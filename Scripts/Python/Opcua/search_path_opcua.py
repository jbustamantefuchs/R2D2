from opcua import Client

# ---------------------------
# Configuración
# ---------------------------
ENDPOINT = "opc.tcp://r2d2:4840/r2d2/server/"
TAG_TO_FIND = "MoveString"

# ---------------------------
# Función para buscar nodos recursivamente
# ---------------------------
def find_node_by_name(node, target_name, path=""):
    results = []
    children = node.get_children()
    for child in children:
        name = child.get_display_name().Text
        child_path = f"{path}/{name}" if path else name
        if name == target_name:
            results.append({"nodeid": child.nodeid.to_string(), "path": child_path})
        # Buscar recursivamente
        results.extend(find_node_by_name(child, target_name, child_path))
    return results

# ---------------------------
# Conexión al servidor OPC UA
# ---------------------------
client = Client(ENDPOINT)
client.set_user("Anonymous")  # si tu servidor permite Anonymous
try:
    client.connect()
    print(f"Conectado a {ENDPOINT}")

    root = client.get_root_node()
    results = find_node_by_name(root, TAG_TO_FIND)

    if results:
        for r in results:
            print(f"Tag encontrado: NodeId={r['nodeid']}, Path={r['path']}")
    else:
        print(f"No se encontró ningún tag llamado '{TAG_TO_FIND}'")

finally:
    client.disconnect()
    print("Desconectado del servidor")

