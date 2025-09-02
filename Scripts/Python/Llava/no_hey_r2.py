import os
import random
import pygame
import requests
import time
import paho.mqtt.client as mqtt
import sounddevice as sd
import queue
import json
import cv2
import base64
import threading
from vosk import Model, KaldiRecognizer
import sys

# Rutas y configuraciones
MODEL = "llava:7b" #chatgpt   #change what model you want
OLLAMA_API_URL = "http://localhost:11434/api/generate"
MQTT_BROKER = "localhost"
MQTT_PORT = 1883
MQTT_TOPIC_RESPUESTA = "/r2d2_respuesta" #answer
MQTT_TOPIC_RESPUESTA2 = "/r2d2_respuesta2"  # ready or not
MQTT_TOPIC_PREGUNTA = "/question_r2d2" #question
MQTT_TOPIC_RESPUESTA_PROMPT = "/r2d2_prompt" #question
SOUND_FOLDER = os.path.expanduser("~/jose/r2d2/sounds/respond")
VOSK_MODEL_PATH = os.path.expanduser("~/jose/r2d2/models/vosk-model-small-en-us-0.15") #reconizition voice #change to proper path 

pygame.init()
pygame.mixer.init()
lista_sonidos = [os.path.join(SOUND_FOLDER, f) for f in os.listdir(SOUND_FOLDER) if f.lower().endswith(".mp3")]

mqtt_queue = queue.Queue()

def on_message(client, userdata, msg):
    pregunta = msg.payload.decode()
    mqtt_queue.put(pregunta)

mqtt_client = mqtt.Client()
mqtt_client.on_message = on_message
mqtt_client.connect(MQTT_BROKER, MQTT_PORT, 60)
mqtt_client.subscribe(MQTT_TOPIC_PREGUNTA)
mqtt_client.loop_start()

audio_queue = queue.Queue()
model = Model(VOSK_MODEL_PATH)

def callback(indata, frames, time_, status):
    if status:
        print(f"{status}")
    audio_queue.put(bytes(indata))

def escuchar_audio():
    print("\nWaiting for task ('exit' to finish)...")
    mqtt_client.publish(MQTT_TOPIC_RESPUESTA2, "Ready for Task")
    with sd.RawInputStream(samplerate=16000, blocksize=8000, dtype='int16', channels=1, callback=callback):
        rec = KaldiRecognizer(model, 16000)
        while True:
            if not mqtt_queue.empty():
                return None  # Cancelar si llegó MQTT
            data = audio_queue.get()
            if rec.AcceptWaveform(data):
                result = json.loads(rec.Result())
                text = result.get("text", "").strip().lower()
                if text:
                    return text

def reproducir_sonido():
    if not lista_sonidos:
        print("No .mp3.")
        return
    archivo = random.choice(lista_sonidos)
    try:
        pygame.mixer.music.stop()
        pygame.mixer.music.load(archivo)
        pygame.mixer.music.set_volume(1.0)
        pygame.mixer.music.play()
        while pygame.mixer.music.get_busy():
            pygame.time.Clock().tick(10)
    except pygame.error as e:
        print(f"Error with sound: {e}")

def convertir_imagen_a_base64(imagen_path): #code
    with open(imagen_path, "rb") as img_file:
        return base64.b64encode(img_file.read()).decode("utf-8")

def tomar_foto(): # take picture
    url = "http://127.0.0.1:5000/video_feed"
    cap = cv2.VideoCapture(url)
    if not cap.isOpened():
        print("Can't open video stream")
        return None

    ret, frame = cap.read()
    cap.release()

    if not ret:
        print("Can't capture frame from stream")
        return None

    carpeta = "/home/student/jose/r2d2/python/llava/pics" # change for the correct
    if not os.path.exists(carpeta):
        os.makedirs(carpeta)

    filename = os.path.join(carpeta, "captured_image.jpg")
    cv2.imwrite(filename, frame)

    return filename


def preguntar_ollama(pregunta, imagen_path=None):
    prompt_inicial = (
        "Act as R2-D2, the astromech droid from the Star Wars saga. "
        "Answer my questions and comments using the information that R2-D2 would have available from his experiences in the films. "
        "Do not make sounds like Beep, Boop, Bip, or any other, only use text. "
        "You must be concise, and use the information you saw in the movies. "
        "Never answer like As R2-D2, I would say: or related, answer directly like him"
        "If you understood, just answer the questions and dont answer the instructions"                                                                                                                            
        "Keep answers short and direct.\n\n"
    )

    if imagen_path:
        imagen_base64 = convertir_imagen_a_base64(imagen_path)
        payload = {
            "model": MODEL,
            "prompt": prompt_inicial + pregunta,
            "images": [imagen_base64],
            "stream": False
        }
    else:
        payload = {
            "model": MODEL,
            "prompt": prompt_inicial + pregunta,
            "stream": False
        }

    try:
        response = requests.post(OLLAMA_API_URL, json=payload)
        if response.status_code == 200:
            return response.json().get("response", "").strip()
        else:
            return f"? Error: {response.text}"
    except requests.exceptions.RequestException as e:
        return f"? Error de conexión con Ollama: {e}"

def limpiar_mqtt_queue():
    while not mqtt_queue.empty():
        mqtt_queue.get()

def simular_espera_interactiva_en_paralelo(stop_event):
    frases = [
        "hmmm...",
        "let me see...",
        "let me adjust my camera...",
        "hold on a second...",
        "Almost got it",
        "well...."
    ]
    for frase in frases:
        if stop_event.is_set():
            break
        print(f"R2-D2: {frase}")
        mqtt_client.publish(MQTT_TOPIC_RESPUESTA, frase)
        time.sleep(4)

def main():
    print("R2-D2 active. Waiting Task...")
    while True:
        fuente = None
        pregunta = None

        if not mqtt_queue.empty():
            pregunta = mqtt_queue.get()
            fuente = "mqtt"
        else:
            pregunta = escuchar_audio()
            if pregunta is None:
                continue
            fuente = "voz"

        if pregunta in ["salir", "exit", "quitter", "stop"]:
            break
            
        if pregunta.strip().lower().startswith("huh"):
            continue

        # Publicar mensaje de espera
        if fuente == "mqtt":
            print(f"\nAsk by write: {pregunta}")
            mqtt_client.publish(MQTT_TOPIC_RESPUESTA2, "Wait until the task is finished")
            mqtt_client.publish(MQTT_TOPIC_RESPUESTA, "...")
        else:
            print(f"\nYou: {pregunta}")
            mqtt_client.publish(MQTT_TOPIC_RESPUESTA_PROMPT, f"{pregunta}")
            mqtt_client.publish(MQTT_TOPIC_RESPUESTA2, "Wait until the task is finished")
            mqtt_client.publish(MQTT_TOPIC_RESPUESTA, "...")

        # Manejo de preguntas visuales
        if any(frase in pregunta.lower() for frase in [
            "what do you see",
            "what are you seeing",
            "what can you see",
            "what do you notice",
            "what is in front of you"
        ]):
            imagen_path = tomar_foto()
            if imagen_path is None:
                respuesta = "Sorry, I couldn't access the camera."
                print(f"\nR2-D2: {respuesta}")
                mqtt_client.publish(MQTT_TOPIC_RESPUESTA, respuesta)
                continue

            # Paralelismo: iniciar espera interactiva + procesado
            stop_event = threading.Event()
            espera_thread = threading.Thread(target=simular_espera_interactiva_en_paralelo, args=(stop_event,))
            espera_thread.start()

            respuesta = preguntar_ollama(pregunta, imagen_path)
            stop_event.set()  # detener mensajes de espera
            espera_thread.join()  # esperar a que finalice

            time.sleep(1)  # pausa adicional de 1 segundo
            print(f"\nR2-D2 (imagen): {respuesta}")
            mqtt_client.publish(MQTT_TOPIC_RESPUESTA, respuesta)

        else:
            modo = random.choice(["antes", "despues", "ambos"])
            if modo in ["antes", "ambos"]:
                reproducir_sonido()

            respuesta = preguntar_ollama(pregunta)
            print(f"\nR2-D2: {respuesta}")
            mqtt_client.publish(MQTT_TOPIC_RESPUESTA, respuesta)

            if modo in ["despues", "ambos"]:
                reproducir_sonido()

        limpiar_mqtt_queue()

    mqtt_client.loop_stop()
    mqtt_client.disconnect()
    print("Bye Bye.")


if __name__ == "__main__":
    main()

