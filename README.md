# control-t60

Firmware propio para el **LILYGO T-Embed CC1101 (Plus)**: control standalone del collar de entrenamiento
**Abbidot T60** con la rueda, el botón y la KEY del T-Embed, y menú de ajustes en pantalla. El protocolo se
decodificó de mi propio collar (2-FSK 433.92 MHz, ±12.7 kHz, bit de 104.2 µs).

> Hecho para **mi equipo**. Úsalo solo con un collar propio y con criterio de bienestar animal. Transmitir
> en 433 MHz debe respetar la normativa local.

## Hardware

- T-Embed CC1101 (ESP32-S3, 16 MB flash, 8 MB PSRAM), radio CC1101, pantalla ST7789V 320×170, encoder con
  pulsador, KEY lateral, parlante I2S (MAX98357A), LED WS2812.
- El bus SPI (FSPI) es **compartido** entre el CC1101, el LCD, la SD y el nRF24: el firmware serializa todo
  acceso (nunca dibuja con una TX en curso).

## Qué hace

| Gesto | Acción |
|---|---|
| Wiggle de la rueda (vaivenes rápidos) | Arma / desarma. Confirmación con beep **local** (agudo = armado, grave = desarmado) |
| Girar ≥ 3 clicks (armado) | Rampa de pulsos discretos a la cadencia del collar (~0.94 s), +PASO por pulso. Derecha = shock, izquierda = vibrar |
| 1–2 clicks (armado) | **Zona de seguridad**: no emite nada (solo cuenta para el wiggle) |
| Click / doble click | Shock BAJO / ALTO |
| KEY toque / mantener | Vibrar / tono repetido |
| KEY + botón 1.5 s (desarmado) | Enciende la pantalla y abre el menú |

- **Pantalla apagada por defecto** (backlight y panel en sleep). El menú (LVGL 9) muestra estado, controla la
  **luz** del collar (encender/apagar, modo 1-3) y edita: cadencia, rampa mín/máx (TOPE), vibrar, shock
  BAJO/ALTO, sensibilidad de armado, pausa de gracia, zona de seguridad, paso de rampa, desarme automático
  (on/off) y tiempo de bloqueo (10–600 s; deshabilitado si el desarme automático está apagado). **Guardar** escribe en NVS; KEY / Salir / 60 s inactivo descartan.
- **Seguridad:** arranca siempre desarmado; desarme por inactividad (desactivable); TOPE duro sobre todo nivel; cadencia
  mínima y tope de estímulos por minuto; watchdog que fuerza el CC1101 a IDLE; "soltar" al final de cada pulso.
- **Shock:** doble barrera. `ALLOW_SHOCK` en compilación (techo) + interruptor `shock on|off` **solo por
  serial** (no en la GUI), persistente en NVS. Apagado → todo shock sale como vibrar.

## Estructura

```
lib/t60frame/   tramas T60 (preámbulo, sync, ID, seq/modo/potencia, Sum8) → corridas para el RMT
lib/gesture/    máquina de gestos y armado — lógica pura, sin Arduino
lib/settings/   tabla de ajustes del menú + carga/guardado (NVS detrás de una interfaz)
src/main.cpp    driver CC1101 (TX 2-FSK asíncrono por GDO0 + RMT), encoder PCNT, tarea de TX, UI LVGL, comandos
src/lcd.h       configuración LovyanGFX del ST7789V
test/           pruebas host (Unity): tramas vs referencia, invariantes de seguridad + fuzz + mutación, ajustes
tools/          t60cmd.py (comandos por serial), gen_vectors.py (vectores de prueba)
```

## ID del collar (`.env`)

El collar solo obedece tramas con **su** ID de 2 bytes. Ese ID **no está en el repositorio**: se lee de `.env`
(ignorado por git) al compilar el firmware, vía `tools/build_env.py`.

```sh
cp .env.example .env    # y poner el ID de tu collar: T60_ID=0xHHLL
```

Sin `.env` el firmware no compila (a propósito). Las pruebas host usan un ID de ejemplo (`0x1234`) y no lo
necesitan; las que comparan contra capturas reales (`test/test_frame/vectors_local.h`, generado por
`tools/gen_vectors.py`) se saltan si el archivo no existe.

## Compilar, probar, flashear

Requiere [PlatformIO](https://platformio.org/).

```sh
pio test -e native                 # pruebas host (lógica pura)
pio run -e tembed                  # compilar
pio run -e tembed -t upload        # flashear (auto-reset por USB)
```

Si tras flashear el serial queda mudo, apretar **RST**.

El build graba la versión (`git describe --dirty`) en el firmware —`ver` o `ping` la muestran— y **se niega a
compilar si el checkout está atrás de `origin/main`** (hace un `git fetch` corto antes), para no flashear una
versión vieja.

## Comandos serie (115200, USB CDC)

`python tools/t60cmd.py "<comando>"` detecta el puerto solo (VID 0x303A).

| Comando | Qué hace |
|---|---|
| `ping`, `ver`, `regs` | estado del CC1101 + versión del firmware / registros vs datasheet |
| `gstat`, `log` | parámetros y estado / registro entrada→decisión→trama (incluye duración de cada TX) |
| `set <campo> <valor>` | ajuste en vivo (no se guarda) |
| `menu [save\|exit]`, `nvs [clear]` | menú y ajustes guardados |
| `shock [on\|off]` | interruptor de shock |
| `tx <luz\|tono\|vibrar\|shock> <0-99>` | una pulsación de prueba (mide y decodifica lo transmitido en el pad) |
| `cap [s]`, `beep`, `lcd`, `wdtest` | medición de entradas, parlante, pantalla, watchdog |

## Licencia

[MIT](LICENSE) © 2026 Daniel Riquelme

## Botón remoto BLE (un solo botón de emergencia)

El T-Embed expone un servidor BLE cifrado (NimBLE, PIN aleatorio por emparejamiento que aparece en su pantalla). Un botón remoto manda solo flancos
(apretado/suelto) y la máquina de gestos decide: **1 toque = shock/vibrar bajo, 2 = alto, 3+ = rampa** (un toque por
pulso). No arma ni desarma (eso sigue siendo el wiggle de la rueda); si el enlace cae se descartan los toques
pendientes. Pruebas desde el Mac: `tools/remote-mac/README.md`. Serial: `ble` (estado), `ble clear` (olvidar emparejados).

### Emparejamiento (ventana local) y control remoto sin teclado

Fuera de una **ventana de emparejamiento** nadie puede emparejar: el passkey es aleatorio y no se muestra; los ya
emparejados reconectan con su clave sin PIN. La ventana dura 60 s, se cierra sola al emparejar y se abre así:

- **Control remoto sin teclado:** DESARMADO, mantener **KEY 2 s** (o `ble pair remoto` por serial). La pantalla dice
  "Emparejar control". Usa el **passkey compartido** `BLE_REMOTE_PIN` de `.env` (patrón oficial
  `Server_secure_static_passkey` / `Client_secure_static_passkey` del core Arduino-ESP32).
- **Mac/celular con teclado:** `ble pair pc` por serial; el PIN aleatorio (nuevo por intento) aparece en la pantalla.

Firmware del control (ESP32, cliente BLE): copiar `Client_secure_static_passkey` del core: `setPassKey(true, <BLE_REMOTE_PIN>)`,
`setCapability(ESP_IO_CAP_IN)` y `setAuthenticationMode(true, true, true)`; el servicio es `6e7a0001-b5a3-f393-e0a9-e50e24dcca9e`.
Tras el primer emparejamiento queda la clave (bond) y el passkey ya no interviene; para rotarlo, cambiar `.env` en ambos firmwares.
