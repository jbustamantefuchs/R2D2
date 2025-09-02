import rospy
from geometry_msgs.msg import Twist
from std_msgs.msg import String

def process_and_publish(data):
    try:
        # Extraer direction y speed desde el mensaje recibido
        if data.data:
            msg = data.data.strip().replace("(", "").replace(")", "")
            direction, speed = map(int, msg.split(','))

        else:
            raise ValueError("Datos vacíos")

        twist_msg = Twist()
        twist_msg.linear.x = 0.0  # Valor base por defecto
        twist_msg.linear.y = 0.0
        twist_msg.angular.z = 0.0
        
        if direction == 1:  # Diagonal Right Forward
            twist_msg.linear.x = 0.5
            twist_msg.linear.y = -0.5
        elif direction == 2:  # Diagonal Right Backward
            twist_msg.linear.x = -0.5
            twist_msg.linear.y = -0.5
        elif direction == 3:  # Diagonal Left Forward
            twist_msg.linear.x = 0.5
            twist_msg.linear.y = 0.5
        elif direction == 4:  # Diagonal Left Backward
            twist_msg.linear.x = -0.5
            twist_msg.linear.y = 0.5
        elif direction == 5:  # Right
            twist_msg.linear.y = -0.5
        elif direction == 6:  # Left
            twist_msg.linear.y = 0.5
        elif direction == 7:  # Backward
            twist_msg.linear.x = -0.5
        elif direction == 8:  # Forward
            twist_msg.linear.x = 0.5
        elif direction == 9:  # Right Turn
            twist_msg.angular.z = -0.5
        elif direction == 10: # Left Turn
            twist_msg.angular.z = 0.5
        
        # Ajustar velocidad en función de speed
        if speed > 4:
            increment = 0.1 * ((speed - 4) // 2)  # Incremento negativo
            if twist_msg.linear.x < 0:
                twist_msg.linear.x -= increment  # Restar incremento si x es negativo
            else:
                twist_msg.linear.x += increment  # Mantener incremento positivo para adelante

            if twist_msg.linear.y < 0:
                twist_msg.linear.y -= increment  # Restar incremento si y es negativo
            else:
                twist_msg.linear.y += increment  # Mantener incremento positivo para adelante

            if twist_msg.angular.z < 0:
                twist_msg.angular.z -= increment  # Restar incremento si z es negativo
            else:
                twist_msg.angular.z += increment  # Mantener incremento positivo para adelante
        
        rospy.loginfo("Publishing to /cmd_vel: {}".format(twist_msg))  # Log the Twist message
        pub_cmd_vel.publish(twist_msg)
        
    except ValueError:
        rospy.logwarn("Formato de mensaje inválido: {}".format(data.data))

def callback_direction_n_speed(data):
    process_and_publish(data)

def callback_nr_controller(data):
    process_and_publish(data)

rospy.init_node('direction_speed_to_cmdvel')
pub_cmd_vel = rospy.Publisher('/cmd_vel', Twist, queue_size=10)
sub_cmd_vel = rospy.Subscriber('/direction_n_speed', String, callback_direction_n_speed)
sub_nr_controller = rospy.Subscriber('/nr_controller', String, callback_nr_controller)

rospy.spin()

