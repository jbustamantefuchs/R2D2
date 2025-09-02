import rospy
from std_msgs.msg import String
import serial

# Configura el puerto serial (ajústalo según tu sistema)
ser = serial.Serial('/dev/ttyACM1', 9600, timeout=1)

def callback(data):
    msg = data.data
    rospy.loginfo(f"Enviando al Arduino: {msg}")
    ser.write((msg + '\n').encode())

# Inicializa el nodo ROS
rospy.init_node('ros_to_arduino_serial')

# Suscríbete a ambos tópicos
rospy.Subscriber('/nr_controller', String, callback)
rospy.Subscriber('/direccion_n_speed', String, callback)

# Mantén el nodo activo
rospy.spin()

