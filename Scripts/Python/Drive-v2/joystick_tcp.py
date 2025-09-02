#!/usr/bin/env python
import socket
import pygame
import time

# Configuración del servidor TCP
TCP_IP = '127.0.0.1'  # Cambia por IP de tu servidor
TCP_PORT = 5005       # Cambia por el puerto de tu servidor

def main():
    # Inicializar socket TCP
    client_socket = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
    try:
        client_socket.connect((TCP_IP, TCP_PORT))
        print(f"Conectado al servidor TCP {TCP_IP}:{TCP_PORT}")
    except Exception as e:
        print(f"No se pudo conectar al servidor TCP: {e}")
        return

    pygame.init()
    pygame.joystick.init()

    if pygame.joystick.get_count() == 0:
        print("No se detectó ningún joystick.")
        return

    joystick = pygame.joystick.Joystick(0)
    joystick.init()
    print(f"Joystick conectado: {joystick.get_name()}")

    speed = 0
    threshold = 0.5
    threshold2 = 0.1
    last_speed_change_time = 0
    speed_debounce_delay = 0.1  # 100 ms debounce para botones de velocidad

    try:
        while True:
            pygame.event.pump()
            current_time = time.time()

            axis_0 = joystick.get_axis(0)
            axis_1 = joystick.get_axis(1)
            axis_2 = joystick.get_axis(2)

            # Control de velocidad con debounce basado en tiempo
            if joystick.get_button(7) and (current_time - last_speed_change_time) > speed_debounce_delay:
                speed += 2
                print(f"Velocidad aumentada: {speed}")
                last_speed_change_time = current_time
            elif joystick.get_button(6) and (current_time - last_speed_change_time) > speed_debounce_delay:
                speed = max(0, speed - 2)
                print(f"Velocidad reducida: {speed}")
                last_speed_change_time = current_time

            # Determinar dirección
            direction = 0
            if abs(axis_2) > threshold2:
                direction = 9 if axis_2 > 0 else 10
            elif abs(axis_0) > threshold or abs(axis_1) > threshold:
                if axis_0 > threshold and axis_1 > threshold:
                    direction = 7
                elif axis_0 > threshold and axis_1 < -threshold:
                    direction = 5
                elif axis_0 < -threshold and axis_1 > threshold:
                    direction = 8
                elif axis_0 < -threshold and axis_1 < -threshold:
                    direction = 6
                elif axis_0 > threshold:
                    direction = 3
                elif axis_0 < -threshold:
                    direction = 4
                elif axis_1 > threshold:
                    direction = 2
                elif axis_1 < -threshold:
                    direction = 1

            msg = f"({direction},{speed})\n"

            # Enviar siempre para máxima respuesta
            try:
                client_socket.sendall(msg.encode('utf-8'))
                print(f"Enviado: {msg.strip()}")
            except Exception as e:
                print(f"Error al enviar datos: {e}")
                break

            time.sleep(0.005)  # Loop rápido (~200 Hz)

    except KeyboardInterrupt:
        print("Programa interrumpido por el usuario.")
    finally:
        client_socket.close()
        pygame.quit()
        print("Conexión cerrada y programa finalizado.")

if __name__ == '__main__':
    main()

