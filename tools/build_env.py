# Script `pre:` de PlatformIO para el firmware (env tembed):
#   1. T60_ID desde `.env` (no versionado): el ID del collar no queda en el repositorio. Formato T60_ID=0x1234.
#   2. FW_VERSION = `git describe --always --dirty` grabado en el firmware (`ver` / `ping`): permite comparar lo
#      que corre en el T-Embed con el repo.
#   3. Se niega a compilar si este checkout está ATRASADO respecto de su upstream (origin/main), tras un
#      `git fetch` corto (si no hay red, se usa lo último conocido): así nunca se flashea una versión vieja.
import os
import re
import subprocess

Import("env")  # noqa: F821 (lo inyecta PlatformIO)

proj = env.subst("$PROJECT_DIR")  # noqa: F821


def git(*args, timeout=5):
    try:
        r = subprocess.run(["git", "-C", proj, *args], capture_output=True, text=True, timeout=timeout)
        return r.stdout.strip() if r.returncode == 0 else None
    except (OSError, subprocess.TimeoutExpired):
        return None


# --- 1. ID del collar ---
path = os.path.join(proj, ".env")
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

# --- 3. no compilar una versión atrasada ---
git("fetch", "--quiet", timeout=8)
behind = git("rev-list", "--count", "HEAD..@{upstream}")
if behind is None:
    print("control-t60: AVISO sin upstream (o sin git): no se puede verificar que esté al día con origin/main")
elif int(behind) > 0:
    print(f"\nERROR: este checkout está {behind} commit(s) ATRÁS de {git('rev-parse', '--abbrev-ref', '@{upstream}')}."
          f"\nActualizá antes de flashear:  git -C {proj} pull --ff-only\n")
    env.Exit(1)  # noqa: F821

# --- 2. versión ---
ver = git("describe", "--always", "--dirty", "--abbrev=7") or "sin-git"
env.Append(CPPDEFINES=[("T60_ID", val), ("FW_VERSION", env.StringifyMacro(ver))])  # noqa: F821
print(f"control-t60: T60_ID desde .env, versión {ver}")
