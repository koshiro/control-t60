# Botón remoto BLE desde el Mac

App local (Web Bluetooth) para probar el botón remoto del T-Embed. Requiere **Chrome o Edge** (Safari no soporta Web Bluetooth).

```bash
python3 -m http.server 8765 --directory tools/remote-mac   # y abrir http://localhost:8765
```

1. En el T-Embed: armar con el wiggle de la rueda (el remoto no arma).
2. **Conectar** → elegir "T60 control". La primera vez macOS pide el PIN de 6 dígitos (`BLE_PIN` del `.env`).
3. Mantener el botón (o la barra espaciadora): 1 toque = bajo, 2 = alto, 3+ = rampa.

Protocolo GATT (servicio `6e7a0001-b5a3-f393-e0a9-e50e24dcca9e`, todo cifrado):
- `6e7a0002-…` escritura sin respuesta, 1 byte: `1` apretado, `0` suelto.
- `6e7a0003-…` lectura + notificación, 4 bytes: `[armado, shock, nivel de rampa, nº de pulsos]`.

Si el Mac recuerda un emparejamiento viejo y falla, olvidar el dispositivo en Ajustes → Bluetooth y `ble clear` por serial.
