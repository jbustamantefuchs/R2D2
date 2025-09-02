import serial
import time
import socket
import threading

puerto = '/dev/ttyACM0'  # Cambia según tu sistema
baudrate = 9600
host = '0.0.0.0'         # Acepta conexiones en todas las interfaces
puertos_sockets = [5005, 8821]  # Escuchar en ambos puertos

def enviar_comando(ser, comando):
    comando_str = comando + '\n'
    ser.write(comando_str.encode('utf-8'))
    print(f"Enviado al Arduino: {comando}")

def leer_respuesta_inmediata(ser, timeout=1):
    tiempo_inicio = time.time()
    while time.time() - tiempo_inicio < timeout:
        if ser.in_waiting > 0:
            linea = ser.readline().decode('utf-8').strip()
            if linea:
                print(f"Respuesta Arduino: {linea}")

def manejar_cliente(conexion, direccion, ser):
    print(f"Cliente conectado desde {direccion}")
    buffer = ""

    with conexion:
        while True:
            try:
                datos = conexion.recv(1024)
                if not datos:
                    print("Cliente desconectado.")
                    break

                buffer += datos.decode('utf-8')

                # Buscar todos los mensajes completos
                mensajes = []
                while "(" in buffer and ")" in buffer:
                    start = buffer.find("(")
                    end = buffer.find(")", start)
                    if end == -1:
                        break
                    mensaje = buffer[start:end+1]
                    buffer = buffer[end+1:]
                    mensajes.append(mensaje.strip())

                if mensajes:
                    if len(mensajes) >= 3:
                        # Más de 3 mensajes pegados ? enviar (0,0)
                        comando = "(0,0)"
                        enviar_comando(ser, comando)
                        leer_respuesta_inmediata(ser)
                        print(f"Se recibieron {len(mensajes)} mensajes en cadena, enviado: {comando}")
                    else:
                        # 1 o 2 mensajes ? procesarlos normalmente
                        for payload in mensajes:
                            print(f"Recibido: {payload}")
                            try:
                                contenido = payload[1:-1].split(',')
                                if len(contenido) == 2:
                                    dir_num = int(contenido[0])
                                    spd_num = int(contenido[1])
                                    comando = f"({dir_num},{spd_num})"
                                    enviar_comando(ser, comando)
                                    leer_respuesta_inmediata(ser)
                                else:
                                    print("Formato incorrecto, se esperan dos valores.")
                            except ValueError:
                                print("Valores no válidos.")

            except ConnectionResetError:
                print("Conexión reiniciada por el cliente.")
                break




def escuchar_puerto(ser, puerto_socket):
    """Hilo que abre un socket y acepta clientes en el puerto dado"""
    with socket.socket(socket.AF_INET, socket.SOCK_STREAM) as s:
        s.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
        s.bind((host, puerto_socket))
        s.listen(1)
        print(f"Servidor socket escuchando en {host}:{puerto_socket}")

        while True:
            conn, addr = s.accept()
            hilo = threading.Thread(target=manejar_cliente, args=(conn, addr, ser))
            hilo.daemon = True
            hilo.start()

def main():
    with serial.Serial(puerto, baudrate, timeout=0.1) as ser:
        time.sleep(2)  # Espera por reinicio del Arduino

        # Crear un hilo para cada puerto
        for p in puertos_sockets:
            hilo = threading.Thread(target=escuchar_puerto, args=(ser, p))
            hilo.daemon = True
            hilo.start()

        # Mantener vivo el hilo principal
        while True:
            time.sleep(1)

if __name__ == '__main__':
    main()

