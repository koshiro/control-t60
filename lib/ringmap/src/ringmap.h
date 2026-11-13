// Anillo Ring01 (control remoto BLE HID de 3 botones) → clicks de botón. Lógica PURA, sin Arduino ni BLE, probada
// en el host. El anillo no manda "botón N": manda reportes HID (teclas multimedia, o secuencias de mouse/táctil
// guionadas) que además CAMBIAN según su modo (TikTok / música / foto). Por eso el binding se APRENDE: cada
// (modo, botón) deja una "firma" = hash de la ráfaga de reportes que produce un click. Las firmas de los tres modos
// apuntan al mismo botón, así que el modo del anillo no cambia lo que hace (pedido de Daniel: evita errores humanos).
//
//   reporte  = (handle de la característica, bytes). Los reportes todo-ceros ("soltar") se ignoran.
//   ráfaga   = reportes seguidos separados por menos de kBurstGapUs; se cierra tras ese silencio.
//   decode() = parte la ráfaga en firmas conocidas (la más larga primero). Si queda CUALQUIER trozo sin reconocer,
//              no sale nada: una ráfaga desconocida (pulsación larga, otro gesto) nunca se convierte en estímulo.
#pragma once
#include <stddef.h>
#include <stdint.h>

namespace ringmap {

constexpr int kButtons = 3;
constexpr int kMaxSigs = 24;      // 3 modos × 3 botones × (click + doble distinto) = 18 como mucho
constexpr int kMaxBurst = 48;     // reportes por ráfaga; más que eso → ráfaga descartada
constexpr uint32_t kBurstGapUs = 150000;  // PROVISIONAL: silencio que cierra una ráfaga (afinar con `ring log`)
constexpr uint8_t kVersion = 2;

struct Sig {
  uint32_t hash;   // hash de la secuencia de reportes
  uint8_t n;       // cantidad de reportes de la ráfaga
  uint8_t button;  // 0..2
  uint8_t clicks;  // 1, o 2 si el anillo manda un reporte PROPIO para su doble toque
  uint8_t pad;
  uint32_t first;  // hash del primer reporte (para saber si una ráfaga corta puede ser el comienzo de esta)
};

struct Table {  // POD: se guarda tal cual en NVS
  uint8_t version;
  uint8_t count;
  uint8_t pad[2];
  Sig sigs[kMaxSigs];
};

void clear(Table &t);
bool valid(const Table &t);  // versión, cantidad y cada firma dentro de rango (una NVS corrupta no pasa)

bool isRelease(const uint8_t *data, size_t len);                    // todo ceros (o vacío)
uint32_t reportHash(uint16_t src, const uint8_t *data, size_t len);
uint32_t seqHash(const uint32_t *h, int n);

// Parte la ráfaga h[0..n) en firmas. Escribe en `btn` el botón de cada click (hasta maxOut) y devuelve cuántos
// clicks son; 0 si algún trozo no se reconoce o no hay firmas.
int decode(const Table &t, const uint32_t *h, int n, uint8_t *btn, int maxOut);

// ¿La ráfaga ya se puede decidir SIN esperar el silencio? Sí cuando se reconoce entera y su último trozo no puede
// ser el comienzo de una firma más larga (ninguna firma más larga empieza con ese mismo primer reporte). Quita los
// kBurstGapUs de espera en el caso común (botón = un solo reporte). Si no, se espera a closed().
bool settled(const Table &t, const uint32_t *h, int n);

enum class Learn : uint8_t {
  Added,     // firma nueva guardada
  Known,     // ya se reconocía como ese botón y esa cantidad de clicks (p. ej. doble = 2 × simple): nada que guardar
  Conflict,  // se reconoce como OTRO botón/cantidad: no se guarda (los botones no serían distinguibles)
  Empty,     // no llegó ningún reporte
  Full,      // tabla llena o ráfaga demasiado larga
};
Learn learn(Table &t, const uint32_t *h, int n, uint8_t button, uint8_t clicks);

// Junta reportes en ráfagas. add() por cada reporte no-cero; closed() dice si la ráfaga terminó (silencio).
class Burst {
 public:
  void add(uint32_t h, uint32_t nowUs);
  bool closed(uint32_t nowUs) const { return n_ > 0 && (uint32_t)(nowUs - last_) >= kBurstGapUs; }
  bool empty() const { return n_ == 0; }
  bool overflow() const { return overflow_; }
  int size() const { return overflow_ ? 0 : n_; }
  const uint32_t *data() const { return h_; }
  void reset() { n_ = 0; overflow_ = false; }

 private:
  uint32_t h_[kMaxBurst] = {};
  uint32_t last_ = 0;
  int n_ = 0;
  bool overflow_ = false;
};

}  // namespace ringmap
