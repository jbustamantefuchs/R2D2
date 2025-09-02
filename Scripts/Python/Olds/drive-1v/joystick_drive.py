#!/usr/bin/env python
import rospy
from std_msgs.msg import String
import pygame

def main():
    rospy.init_node('joystick_controller', anonymous=True)
    pub = rospy.Publisher('direction_n_speed', String, queue_size=10)

    pygame.init()
    pygame.joystick.init()

    if pygame.joystick.get_count() == 0:
        rospy.logerr("No se detectó ningún joystick.")
        return

    joystick = pygame.joystick.Joystick(0)
    joystick.init()
    rospy.loginfo(f"Joystick conectado: {joystick.get_name()}")

    speed = 0
    prev_direction = -1  # Diferente de 0 para que envíe la primera vez

    try:
        while not rospy.is_shutdown():
            pygame.event.pump()

            axis_0 = joystick.get_axis(0)
            axis_1 = joystick.get_axis(1)
            axis_2 = joystick.get_axis(2)

            # Botones de control de velocidad
            if joystick.get_button(7):  # Botón de aumentar
                speed += 2
                rospy.loginfo(f"Velocidad aumentada: {speed}")
            elif joystick.get_button(6):  # Botón de disminuir
                speed = max(0, speed - 2)
                rospy.loginfo(f"Velocidad reducida: {speed}")

            # Dirección
            direction = 0
            threshold = 0.5

            if abs(axis_2) > threshold:
                direction = 9 if axis_2 > 0 else 10
            elif abs(axis_0) > threshold or abs(axis_1) > threshold:
                if axis_0 > threshold and axis_1 > threshold:
                    direction = 1
                elif axis_0 > threshold and axis_1 < -threshold:
                    direction = 2
                elif axis_0 < -threshold and axis_1 > threshold:
                    direction = 3
                elif axis_0 < -threshold and axis_1 < -threshold:
                    direction = 4
                elif axis_0 > threshold:
                    direction = 5
                elif axis_0 < -threshold:
                    direction = 6
                elif axis_1 > threshold:
                    direction = 7
                elif axis_1 < -threshold:
                    direction = 8

            # Solo publica si cambia la dirección
            if direction != prev_direction:
                msg = f"({direction},{speed})"
                pub.publish(msg)
                rospy.loginfo(f"Enviado: {msg}")
                prev_direction = direction

            rospy.sleep(0.01)  # Pequeña pausa para liberar CPU

    except rospy.ROSInterruptException:
        pass
    finally:
        pygame.quit()
        rospy.loginfo("Programa finalizado.")

if __name__ == '__main__':
    main()

