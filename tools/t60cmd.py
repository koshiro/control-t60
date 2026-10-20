#!/usr/bin/env python3
"""Manda un comando al firmware control-t60 por USB CDC e imprime la respuesta.
Uso: t60cmd.py "<comando>" [--port /dev/cu.usbmodemXXXX] [--wait 3]
Sin --port busca el ESP32-S3 por VID 0x303A (el número de usbmodem cambia; el Meshtastic Heltec no es 0x303A)."""
import argparse, sys, time
import serial
from serial.tools import list_ports


def find_port():
    ports = [p.device for p in list_ports.comports() if p.vid == 0x303A and p.device.startswith("/dev/cu.")]
    if len(ports) != 1:
        sys.exit(f"ERROR: esperaba 1 T-Embed (VID 0x303A), encontré {ports or 'ninguno'}; usa --port")
    return ports[0]


ap = argparse.ArgumentParser()
ap.add_argument("cmd"); ap.add_argument("--port")
ap.add_argument("--wait", type=float, default=3.0, help="segundos de silencio para terminar")
a = ap.parse_args()
s = serial.Serial(a.port or find_port(), 115200, timeout=0.1)
time.sleep(0.2); s.reset_input_buffer()
s.write((a.cmd + "\n").encode())
buf, last = b"", time.time()
while time.time() - last < a.wait:
    d = s.read(4096)
    if d: buf += d; last = time.time()
sys.stdout.write(buf.decode(errors="replace"))
