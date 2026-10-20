# Script `pre:` de PlatformIO: lee T60_ID de `.env` (no versionado) y lo define al compilar el firmware.
# Así el ID del collar no queda en el repositorio. Formato: T60_ID=0x1234 (4 dígitos hex).
import os
import re

Import("env")  # noqa: F821 (lo inyecta PlatformIO)

path = os.path.join(env.subst("$PROJECT_DIR"), ".env")  # noqa: F821
val = None
if os.path.isfile(path):
    with open(path) as f:
        for line in f:
            m = re.match(r"\s*T60_ID\s*=\s*(0x[0-9A-Fa-f]{4})\s*(#.*)?$", line)
            if m:
                val = m.group(1)
if not val:
    print(f"\nERROR: falta T60_ID en {path} (copiá .env.example a .env y poné el ID de tu collar)\n")
    env.Exit(1)  # noqa: F821
env.Append(CPPDEFINES=[("T60_ID", val)])  # noqa: F821
print("control-t60: T60_ID desde .env")
