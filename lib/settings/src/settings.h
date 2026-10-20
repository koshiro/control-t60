// Ajustes editables del control T60 (menú en pantalla + NVS) — lógica PURA, sin Arduino, probada en el host.
// Una tabla de campos (clave NVS, etiqueta, unidad, rango, paso) sobre gesture::Config. El menú y la persistencia
// solo pasan por aquí, así que todo valor que entra a la máquina (del menú o de una NVS corrupta/vieja) queda
// dentro de rango y con los niveles <= TOPE. La E/S real (Preferences) va detrás de `Store`.
#pragma once
#include <stdint.h>

#include "gesture.h"

namespace settings {

enum class Unit : uint8_t { Ms, Level, Count, Clicks, Sec, Bool };  // Bool = interruptor en el menú (0/1)

struct Field {
  const char *key;    // clave NVS (<= 15 caracteres)
  const char *label;  // etiqueta del menú (ASCII: las fuentes de LVGL no traen acentos)
  Unit unit;
  int32_t min, max, step;  // en unidades de pantalla (ms o nivel)
};

constexpr int kCount = 12;
constexpr int32_t kSchema = 1;  // sube si cambia el significado de una clave → lo guardado se ignora

const Field &field(int i);
int32_t get(const gesture::Config &c, int i);           // valor del campo i en unidades de pantalla
void set(gesture::Config &c, int i, int32_t v);         // recorta a [min,max], ajusta al paso y escribe
int32_t clampField(int i, int32_t v);
// ¿El campo i aplica con esta config? (p.ej. "Tiempo bloqueo" solo con auto-desarme activado). El menú muestra
// deshabilitados los que no aplican y el foco del encoder los salta; su valor se conserva y se sigue guardando.
bool enabled(const gesture::Config &c, int i);
// Recorta todos los campos a su rango y aplica la regla del TOPE (BAJO/ALTO/mín/vibrar/tono <= cap), igual que
// Machine::reset(): lo que muestra el menú es exactamente lo que usará la máquina y lo que se guarda.
void normalize(gesture::Config &c);

struct Store {
  virtual bool get(const char *key, int32_t &v) = 0;  // false si la clave no existe
  virtual bool put(const char *key, int32_t v) = 0;
  virtual ~Store() {}
};

// Parte de `base` (defaults) y superpone lo guardado si el esquema coincide; siempre devuelve normalizado.
// `found` = cuántos campos venían de la NVS.
gesture::Config load(Store &s, const gesture::Config &base, int *found = nullptr);
bool save(Store &s, const gesture::Config &c);  // guarda la versión normalizada + el esquema

// Modo de luz del collar (potencia del modo 0x41; 0 = apagar no es un modo). Solo el 1 está verificado en el
// collar; 2 y 3 son la suposición de collar-re.md. Se guarda aparte de gesture::Config (no es un gesto).
constexpr int32_t kLightModeMin = 1, kLightModeMax = 3;
int32_t clampLightMode(int32_t v);
int32_t loadLightMode(Store &s);  // 1 si no hay nada guardado o el esquema no coincide
bool saveLightMode(Store &s, int32_t mode);

// Interruptor de shock persistente (se cambia solo por serial). Independiente del esquema: se guarda al cambiarlo,
// aunque nunca se haya guardado el menú. Cualquier valor que no sea exactamente 1 se lee como apagado.
bool loadShock(Store &s);
bool saveShock(Store &s, bool on);

}  // namespace settings
