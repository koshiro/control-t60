# Script `pre:` de PlatformIO para el firmware del control remoto (env remote_c3):
#   BLE_REMOTE_PIN desde `.env` (no versionado; el mismo passkey que usa el T-Embed en la ventana de emparejamiento del
#   remoto) y FW_VERSION = `git describe --always --dirty`. No hace `git fetch` (el control no depende de origin).
import os
import re
import subprocess

Import("env")  # noqa: F821 (lo inyecta PlatformIO)

proj = env.subst("$PROJECT_DIR")  # noqa: F821
pin = None
path = os.path.join(proj, ".env")
if os.path.isfile(path):
    with open(path) as f:
        for line in f:
            m = re.match(r"\s*BLE_REMOTE_PIN\s*=\s*([0-9]{6})\s*(#.*)?$", line)
            if m:
                pin = m.group(1)
if not pin:
    print(f"\nERROR: falta BLE_REMOTE_PIN (6 dígitos) en {path} (ver .env.example)\n")
    env.Exit(1)  # noqa: F821
try:
    r = subprocess.run(["git", "-C", proj, "describe", "--always", "--dirty", "--abbrev=7"], capture_output=True, text=True, timeout=5)
    ver = r.stdout.strip() if r.returncode == 0 else "sin-git"
except (OSError, subprocess.TimeoutExpired):
    ver = "sin-git"
env.Append(CPPDEFINES=[("BLE_REMOTE_PIN", pin), ("FW_VERSION", env.StringifyMacro(ver))])  # noqa: F821
print(f"control-t60 remoto: BLE_REMOTE_PIN desde .env, versión {ver}")
