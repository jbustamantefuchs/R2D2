#!/usr/bin/env python3
import rospy
from std_msgs.msg import Int32
import subprocess

estado_anterior = -1  # Estado inicial indefinido

def hablar_con_llava(estilo_mensaje):
    """
    Usa ollama con LLaVA-7B para generar un mensaje con estilo R2-D2 (sin sonidos).
    """
    prompt = (
        f"Actúa como R2-D2 de Star Wars, pero usa solo palabras humanas. "
        f"Nunca uses sonidos como 'beep', 'boop', 'brrt' ni ningún ruido robótico. "
        f"{estilo_mensaje}"
    )

    try:
        respuesta = subprocess.check_output(["ollama", "run", "llava:7b", prompt], text=True)
        print("\n?? R2-D2 dice:", respuesta.strip(), "\n")
    except Exception as e:
        print("?? Error al ejecutar Ollama:", e)

def callback(data):
    global estado_anterior
    estado_actual = data.data

    if estado_actual != estado_anterior:
        if estado_actual == 1:
            print("?? Persona detectada - Activando R2-D2 (saludo)...")
            hablar_con_llava("Saluda con entusiasmo a una persona que acaba de llegar.")
        elif estado_actual == 0:
            print("?? Persona ausente - Activando R2-D2 (despedida)...")
            hablar_con_llava("Despídete con amabilidad porque la persona se ha ido.")

        estado_anterior = estado_actual

# Inicializa ROS
rospy.init_node('r2d2_llava_node')
rospy.Subscriber('/persona_detectada', Int32, callback)

rospy.loginfo("?? R2-D2 silencioso con alma de LLaVA está escuchando...")
rospy.spin()

