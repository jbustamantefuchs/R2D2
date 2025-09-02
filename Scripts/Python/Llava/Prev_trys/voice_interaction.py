import os
import random
import pygame
import requests
import time
import paho.mqtt.client as mqtt
import sounddevice as sd
import queue
import json
from vosk import Model, KaldiRecognizer

# ?? Rutas
MODEL = "llava:7b"
OLLAMA_URL = "http://localhost:11434/api/generate"
MQTT_BROKER = "localhost"
MQTT_PORT = 1883
MQTT_TOPIC_RESPUESTA = "/r2d2_respuesta"
MQTT_TOPIC_PREGUNTA = "/question_r2d2"
SOUND_FOLDER = os.path.expanduser("~/jose/r2d2/sounds/respond")
VOSK_MODEL_PATH = os.path.expanduser("~/jose/r2d2/models/vosk-model-small-en-us-0.15")

# ?? Sonidos
pygame.init()
pygame.mixer.init()
lista_sonidos = [
    os.path.join(SOUND_FOLDER, f)
    for f in os.listdir(SOUND_FOLDER)
    if f.lower().endswith(".mp3")
]

# ?? Cola para preguntas MQTT
mqtt_queue = queue.Queue()

# ?? MQTT Setup
def on_message(client, userdata, msg):
    pregunta = msg.payload.decode()
    mqtt_queue.put(pregunta)

mqtt_client = mqtt.Client()
mqtt_client.on_message = on_message
mqtt_client.connect(MQTT_BROKER, MQTT_PORT, 60)
mqtt_client.subscribe(MQTT_TOPIC_PREGUNTA)
mqtt_client.loop_start()

# ?? Vosk reconocimiento de voz
audio_queue = queue.Queue()
model = Model(VOSK_MODEL_PATH)

def callback(indata, frames, time_, status):
    if status:
        print(f"?? {status}")
    audio_queue.put(bytes(indata))

def escuchar_audio():
    print("\n??? Esperando voz (di 'exit' para salir)...")
    with sd.RawInputStream(samplerate=16000, blocksize=8000, dtype='int16',
                           channels=1, callback=callback):
        rec = KaldiRecognizer(model, 16000)
        while True:
            if not mqtt_queue.empty():
                return None  # ?? Cancelar escucha si llegó MQTT
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

def preguntar_ollama(pregunta):
    prompt_inicial = (
        "Act as R2-D2, the astromech droid from the Star Wars saga. "
        "Answer my questions and comments using the information that R2-D2 would have available from his experiences in the films. "
        "Do not make sounds like Beep, Boop, Bip, or any other, only use text. "
        "You must be concise, and use the information you saw in the movies.\n\n"
    )
    payload = {
        "model": MODEL,
        "prompt": prompt_inicial + pregunta,
        "stream": False
    }
    try:
        response = requests.post(OLLAMA_URL, json=payload)
        if response.status_code == 200:
            respuesta = response.json()["response"].strip()
            mqtt_client.publish(MQTT_TOPIC_RESPUESTA, respuesta)
            return respuesta
        else:
            return f"? Error: {response.text}"
    except requests.exceptions.RequestException as e:
        return f"? Error de conexión con Ollama: {e}"

def limpiar_mqtt_queue():
    while not mqtt_queue.empty():
        mqtt_queue.get()

def main():
    print("?? R2-D2 activo. Esperando preguntas por voz o MQTT...")
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

        if fuente == "mqtt":
            print(f"\n?? Pregunta por MQTT: {pregunta}")
        else:
            print(f"\n??? Tú dijiste: {pregunta}")

        modo = random.choice(["antes", "despues", "ambos"])
        if modo in ["antes", "ambos"]:
            reproducir_sonido()

        respuesta = preguntar_ollama(pregunta)
        print(f"\n?? R2-D2 responde: {respuesta}")

        if modo in ["despues", "ambos"]:
            reproducir_sonido()

        limpiar_mqtt_queue()  # ?? Evita doble respuesta si llega MQTT durante una interacción

    mqtt_client.loop_stop()
    mqtt_client.disconnect()
    print("?? Adiós.")

if __name__ == "__main__":
    main()

