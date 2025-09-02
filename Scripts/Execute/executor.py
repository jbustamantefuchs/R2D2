import subprocess
import time
import os

# Lista de scripts Python
scripts = [
    "/home/student/jose/r2d2/python/video/multiple_feed.py",
    "/home/student/jose/r2d2/python/drive-v2/send_ard_from_socket.py",
    "/home/student/jose/r2d2/python/llava/no_hey_r2.py",
    "/home/student/jose/r2d2/python/actions/actions.py",
    "/home/student/jose/r2d2/python/drive-v2/joystick_tcp.py"
]

# Script bash final
bash_script = "/home/student/jose/r2d2/python/detectnet/head/run.sh"

# Verificar que los archivos existen
scripts = [s for s in scripts if os.path.isfile(s)]
if not scripts:
    print("No hay scripts Python válidos.")
    exit(1)

if not os.path.isfile(bash_script):
    print("No se encontró el script bash final.")
    bash_script = None

# Abrir Terminator
subprocess.Popen(["terminator"])
time.sleep(2)  # esperar a que el terminal se abra

# Ejecutar el primer script en el panel principal
time.sleep(2)
subprocess.run(["xdotool", "type", "--delay", "50", f"python3 {scripts[0]}"])
subprocess.run(["xdotool", "key", "Return"])
time.sleep(0.5)

# Dividir horizontalmente y ejecutar los demás scripts
for script in scripts[1:]:
    subprocess.run(["xdotool", "key", "ctrl+shift+o"])  # dividir horizontal
    time.sleep(2)
    subprocess.run(["xdotool", "type", "--delay", "50", f"python3 {script}"])
    subprocess.run(["xdotool", "key", "Return"])
    time.sleep(0.5)

# Ejecutar bash final en un nuevo panel horizontal
if bash_script:
    subprocess.run(["xdotool", "key", "ctrl+shift+o"])  # dividir horizontal
    time.sleep(2)
    subprocess.run(["xdotool", "type", "--delay", "50", f"bash {bash_script}"])
    subprocess.run(["xdotool", "key", "Return"])
    
    # Esperar 2 segundos y enviar "changeme"
    time.sleep(2)
    subprocess.run(["xdotool", "type", "--delay", "50", "changeme"])
    subprocess.run(["xdotool", "key", "Return"])

