import subprocess
import time

# Espera opcional antes de cerrar todo
time.sleep(1)

# Mata todos los procesos Python3
try:
    subprocess.run(["pkill", "-f", "python3"])
    print("Todos los procesos Python3 han sido terminados.")
except Exception as e:
    print(f"Error al intentar matar procesos: {e}")

