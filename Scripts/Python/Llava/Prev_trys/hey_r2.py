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

# Rutas y configuraciones
MODEL = "llava:7b"
OLLAMA_API_URL = "http://localhost:11434/api/generate"
MQTT_BROKER = "localhost"
MQTT_PORT = 1883
MQTT_TOPIC_RESPUESTA = "/r2d2_respuesta"
MQTT_TOPIC_PREGUNTA = "/question_r2d2"
SOUND_FOLDER = os.path.expanduser("~/jose/r2d2/sounds/respond")
VOSK_MODEL_PATH = os.path.expanduser("~/jose/r2d2/models/vosk-model-small-en-us-0.15")

pygame.init()
pygame.mixer.init()
lista_sonidos = [os.path.join(SOUND_FOLDER, f) for f in os.listdir(SOUND_FOLDER) if f.lower().endswith(".mp3")]

mqtt_queue = queue.Queue()
audio_queue = queue.Queue()
model = Model(VOSK_MODEL_PATH)

def on_message(client, userdata, msg):
    pregunta = msg.payload.decode()
    mqtt_queue.put(pregunta)

mqtt_client = mqtt.Client()
mqtt_client.on_message = on_message
mqtt_client.connect(MQTT_BROKER, MQTT_PORT, 60)
mqtt_client.subscribe(MQTT_TOPIC_PREGUNTA)
mqtt_client.loop_start()

def callback(indata, frames, time_, status):
    if status:
        print(f"?? {status}")
    audio_queue.put(bytes(indata))

def esperar_activacion():
    print("\n?? Di 'Hey R2' para activar...")
    with sd.RawInputStream(samplerate=16000, blocksize=8000, dtype='int16', channels=1, callback=callback):
        rec = KaldiRecognizer(model, 16000)
        while True:
            data = audio_queue.get()
            if rec.AcceptWaveform(data):
                result = json.loads(rec.Result())
                texto = result.get("text", "").strip().lower()
                if "hey r2" in texto:
                    print("?? Activado por 'Hey R2'")
                    return

def escuchar_audio():
    print("\n??? Esperando voz (di 'exit' para salir)...")
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
        print("?? No se encontraron sonidos .mp3.")
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
        print(f"?? Error al reproducir sonido: {e}")

def convertir_imagen_a_base64(imagen_path):
    with open(imagen_path, "rb") as img_file:
        return base64.b64encode(img_file.read()).decode("utf-8")

def tomar_foto():
    print("?? Capturando foto con cámara...")
    cap = cv2.VideoCapture(4)
    if not cap.isOpened():
        print("?? No se pudo abrir la cámara")
        return None
    ret, frame = cap.read()
    cap.release()
    if not ret:
        print("?? No se pudo capturar la imagen")
        return None
    
    carpeta = "/home/student/jose/r2d2/python/llava/pics"
    if not os.path.exists(carpeta):
        os.makedirs(carpeta)

    filename = os.path.join(carpeta, "captured_image.jpg")
    cv2.imwrite(filename, frame)
    print(f"?? Foto guardada como {filename}")
    return filename

def preguntar_ollama(pregunta, imagen_path=None):
    prompt_inicial = (
        "Act as R2-D2, the astromech droid from the Star Wars saga. "
        "Answer my questions and comments using the information that R2-D2 would have available from his experiences in the films. "
        "Do not make sounds like Beep, Boop, Bip, or any other, only use text. "
        "You must be concise, and use the information you saw in the movies. "
        "Never answer like As R2-D2, I would say: or related, answer directly like him"
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

def limpiar_audio_queue():
    while not audio_queue.empty():
        audio_queue.get()

def simular_espera_interactiva_en_paralelo(stop_event):
    frases = [
        "hmmm...",
        "let me see...",
        "let me adjust my camera...",
        "hold on a second...",
        "Almost got it"
    ]
    for frase in frases:
        if stop_event.is_set():
            break
        print(f"?? R2-D2: {frase}")
        mqtt_client.publish(MQTT_TOPIC_RESPUESTA, frase)
        time.sleep(4)

def main():
    print("?? R2-D2 activo. Esperando activación por voz ('Hey R2') o MQTT...")

    while True:
        fuente = None
        pregunta = None

        if not mqtt_queue.empty():
            pregunta = mqtt_queue.get()
            fuente = "mqtt"
        else:
            esperar_activacion()          # Espera "Hey R2"
            limpiar_audio_queue()         # Limpia audio residual
            pregunta = escuchar_audio()   # Escucha la pregunta después de activación
            if pregunta is None:
                continue
            fuente = "voz"

        if pregunta in ["salir", "exit", "quitter", "stop"]:
            break

        if fuente == "mqtt":
            print(f"\n?? Pregunta por MQTT: {pregunta}")
        else:
            print(f"\n??? Tú dijiste: {pregunta}")

        if "what do you see" in pregunta.lower():
            imagen_path = tomar_foto()
            if imagen_path is None:
                respuesta = "Sorry, I couldn't access the camera."
                print(f"\n?? R2-D2 responde: {respuesta}")
                mqtt_client.publish(MQTT_TOPIC_RESPUESTA, respuesta)
                continue

            stop_event = threading.Event()
            espera_thread = threading.Thread(target=simular_espera_interactiva_en_paralelo, args=(stop_event,))
            espera_thread.start()

            respuesta = preguntar_ollama(pregunta, imagen_path)
            stop_event.set()
            espera_thread.join()

            time.sleep(1)
            print(f"\n?? R2-D2 responde (imagen): {respuesta}")
            mqtt_client.publish(MQTT_TOPIC_RESPUESTA, respuesta)

        else:
            modo = random.choice(["antes", "despues", "ambos"])
            if modo in ["antes", "ambos"]:
                reproducir_sonido()

            respuesta = preguntar_ollama(pregunta)
            print(f"\n?? R2-D2 responde: {respuesta}")
            mqtt_client.publish(MQTT_TOPIC_RESPUESTA, respuesta)

            if modo in ["despues", "ambos"]:
                reproducir_sonido()

        limpiar_mqtt_queue()

    mqtt_client.loop_stop()
    mqtt_client.disconnect()
    print("?? Adiós.")

if __name__ == "__main__":
    main()

