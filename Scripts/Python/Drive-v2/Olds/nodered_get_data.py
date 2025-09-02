import socket

TCP_PORT = 8821
BUFFER_SIZE = 1024

def main():
    server_socket = socket.socket(socket.AF_INET, socket.SOCK_STREAM)

    # ?? Esto permite reusar el puerto sin esperar TIME_WAIT
    server_socket.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)

    server_socket.bind(("0.0.0.0", TCP_PORT))
    server_socket.listen(1)
    print(f"[ OK ] Servidor TCP escuchando en puerto {TCP_PORT}")

    while True:
        conn, addr = server_socket.accept()
        print(f"[ NUEVA CONEXIÓN ] {addr}")

        with conn:
            while True:
                data = conn.recv(BUFFER_SIZE)
                if not data:
                    break
                msg = data.decode("utf-8").strip()
                print(f"[TCP] {msg}")

if __name__ == "__main__":
    main()

