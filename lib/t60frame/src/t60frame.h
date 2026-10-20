// Tramas del collar T60 (Abbidot) — sin dependencias de Arduino, se prueba en el host (pio test -e native).
// Especificación = collar_sub.py frame_bits()/gen() del skill t-embed-cc1101 (protocolo en collar-re.md):
//   preámbulo "10"x523 + "11" + 0x30 + [ID_hi][ID_lo][seq][modo|canal][potencia][chk], MSB primero,
//   chk = Sum8 de los 5 bytes previos. Bit NRZ 104.2 µs, "1" = desviación +. Tras cada trama 11 ms en bajo.
#pragma once
#include <stddef.h>
#include <stdint.h>

namespace t60 {

// ID del collar: NO se versiona. El firmware lo recibe de `.env` (T60_ID=0xHHLL) vía tools/env_id.py; las pruebas
// host usan un ID de ejemplo (-DT60_ID=0x1234 en [env:native]).
#ifndef T60_ID
#error "Falta T60_ID: crea .env con T60_ID=0xHHLL (ver .env.example)"
#endif
constexpr uint8_t kIdHi = (uint8_t)(((T60_ID) >> 8) & 0xFF), kIdLo = (uint8_t)((T60_ID) & 0xFF);
constexpr int kPreambleBits = 1046;
constexpr int kSyncBits = 10;  // "11" + 0x30
constexpr int kFrameBits = kPreambleBits + kSyncBits + 6 * 8;  // 1104
constexpr uint32_t kBitTenthsUs = 1042;    // 104.2 µs
constexpr uint32_t kGapTenthsUs = 110000;  // 11 ms entre tramas

// Tipo en el nibble alto; el canal (1 = collar 1) va en el bajo.
constexpr uint8_t kShock = 0x10, kVibrar = 0x20, kTono = 0x30, kLuz = 0x40;
constexpr uint8_t kReleaseMode = 0x01, kReleasePower = 0xFF;

struct Frame {
  uint8_t seq = 0, mode = 0, power = 0;
  uint8_t idHi = kIdHi, idLo = kIdLo;
  uint8_t chkXor = 0;  // != 0 solo para pruebas negativas (checksum malo a propósito)
};

uint8_t checksum(const Frame &f);
void payload(const Frame &f, uint8_t out[6]);
// Un bit por byte (0/1); `out` debe tener kFrameBits elementos.
void frameBits(const Frame &f, uint8_t *out);

struct Run {
  uint8_t level;      // 1 = desviación +, 0 = desviación −
  uint32_t tenthsUs;  // duración en décimas de µs
};

// Una pulsación como la de collar_sub.py gen(): `frames` tramas iguales y, si `release`, la trama
// "soltar" (seq+1, 0x01, 0xFF) con el mismo ID/chkXor; cada trama seguida de 11 ms en bajo.
// Devuelve la cantidad de corridas, o -1 si no caben en `max`.
int pressRuns(const Frame &f, int frames, bool release, Run *out, int max);

// Una pulsación con potencia VARIABLE (prueba del punto 3): mismo `seq` y `mode` para todas las tramas,
// pero la potencia va cambiando — `perFrame` tramas por cada nivel de `powers`, en orden — y al final la
// trama "soltar" (seq+1) si `release`. Sirve para ver si el collar sigue el cambio de potencia dentro de
// una misma pulsación. Devuelve la cantidad de corridas, o -1 si no caben en `max`.
int rampRuns(uint8_t seq, uint8_t mode, const uint8_t *powers, int nPowers, int perFrame, bool release,
             Run *out, int max);

}  // namespace t60
