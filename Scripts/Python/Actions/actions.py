import paho.mqtt.client as mqtt
import pygame
import os

pygame.mixer.init()
print("[INFO] Reproductor de sonido iniciado.")

# Carpeta base para los sonidos
SOUND_DIR = "/home/student/jose/r2d2/sounds"

# Mapear comandos a archivos
SOUNDS = {
    "14": "Scream.mp3",
    "13":  "Laughing.mp3"
}

def play_sound(name):
    path = os.path.join(SOUND_DIR, name)
    print(f"[INFO] Reproduciendo: {path}")
    pygame.mixer.music.load(path)
    pygame.mixer.music.play()

def on_message(client, userdata, message):
    cmd = message.payload.decode().strip().lower()
    print(f"[MQTT] Comando recibido: {cmd}")
    if cmd in SOUNDS:
        play_sound(SOUNDS[cmd])
    else:
        print(f"[WARNING] Comando no reconocido: {cmd}")

client = mqtt.Client()
client.on_message = on_message

print("[INFO] Conectando al broker MQTT...")
client.connect("localhost", 1883)
client.subscribe("/comando_hacer")
print("[INFO] Suscrito a /comando_hacer. Esperando comandos...")

client.loop_forever()

