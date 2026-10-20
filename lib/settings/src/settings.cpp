#include "settings.h"

#include <string.h>

namespace settings {

// Rangos: la cadencia no baja de 500 ms (medido 2026-10-20: a ≤0.84 s el collar ya pierde pulsos; 0.94 s es el
// punto bueno). Niveles 0-99 = byte de potencia del collar; el TOPE manda sobre el resto (normalize).
static const Field kFields[kCount] = {
  {"gap", "Cadencia", Unit::Ms, 500, 2000, 20},          // minStimGapUs
  {"minr", "Rampa min", Unit::Level, 0, 99, 1},          // minRamp
  {"cap", "Rampa max", Unit::Level, 1, 99, 1},           // cap (TOPE)
  {"vib", "Vibrar", Unit::Level, 0, 99, 1},              // keyVibrPower
  {"lo", "Shock BAJO", Unit::Level, 0, 99, 1},           // lowPower
  {"hi", "Shock ALTO", Unit::Level, 0, 99, 1},           // highPower
  {"wig", "Armado (rev)", Unit::Count, 2, 8, 1},         // wiggleRevs: menos reversas = más sensible
  {"grace", "Pausa gracia", Unit::Ms, 0, 3000, 100},     // armGraceUs
  {"zona", "Zona segur.", Unit::Clicks, 0, 4, 1},        // safeClicks (zona de seguridad de la rueda)
  {"paso", "Paso rampa", Unit::Level, 1, 10, 1},         // step: niveles que sube cada PULSO de la rampa
  {"adis", "Auto-desarme", Unit::Bool, 0, 1, 1},         // autoDisarm: desarme por inactividad on/off
  {"tblq", "Tiempo bloqueo", Unit::Sec, 10, 600, 10},    // autoDisarmUs: inactividad hasta desarmar
};

const Field &field(int i) { return kFields[i]; }

int32_t clampField(int i, int32_t v) {
  const Field &f = kFields[i];
  if (v < f.min) v = f.min;
  if (v > f.max) v = f.max;
  v = f.min + (v - f.min + f.step / 2) / f.step * f.step;  // al paso más cercano
  if (v > f.max) v -= f.step;
  return v;
}

bool enabled(const gesture::Config &c, int i) {
  if (!strcmp(kFields[i].key, "tblq")) return c.autoDisarm;
  return true;
}

int32_t get(const gesture::Config &c, int i) {
  switch (i) {
    case 0: return (int32_t)(c.minStimGapUs / 1000);
    case 1: return c.minRamp;
    case 2: return c.cap;
    case 3: return c.keyVibrPower;
    case 4: return c.lowPower;
    case 5: return c.highPower;
    case 6: return c.wiggleRevs;
    case 7: return (int32_t)(c.armGraceUs / 1000);
    case 8: return c.safeClicks;
    case 9: return c.step;
    case 10: return c.autoDisarm ? 1 : 0;
    case 11: return (int32_t)(c.autoDisarmUs / 1000000);
  }
  return 0;
}

void set(gesture::Config &c, int i, int32_t v) {
  v = clampField(i, v);
  switch (i) {
    case 0: c.minStimGapUs = (uint32_t)v * 1000; break;
    case 1: c.minRamp = (uint8_t)v; break;
    case 2: c.cap = (uint8_t)v; break;
    case 3: c.keyVibrPower = (uint8_t)v; break;
    case 4: c.lowPower = (uint8_t)v; break;
    case 5: c.highPower = (uint8_t)v; break;
    case 6: c.wiggleRevs = (uint8_t)v; break;
    case 7: c.armGraceUs = (uint32_t)v * 1000; break;
    case 8: c.safeClicks = (uint8_t)v; break;
    case 9: c.step = (uint8_t)v; break;
    case 10: c.autoDisarm = v != 0; break;
    case 11: c.autoDisarmUs = (uint32_t)v * 1000000; break;
  }
}

void normalize(gesture::Config &c) {
  for (int i = 0; i < kCount; i++) set(c, i, get(c, i));
  if (c.minRamp > c.cap) c.minRamp = c.cap;
  if (c.keyVibrPower > c.cap) c.keyVibrPower = c.cap;
  if (c.lowPower > c.cap) c.lowPower = c.cap;
  if (c.highPower > c.cap) c.highPower = c.cap;
  if (c.keyTonoPower > c.cap) c.keyTonoPower = c.cap;
}

gesture::Config load(Store &s, const gesture::Config &base, int *found) {
  gesture::Config c = base;
  int n = 0;
  int32_t ver;
  if (s.get("schema", ver) && ver == kSchema) {
    for (int i = 0; i < kCount; i++) {
      int32_t v;
      if (s.get(kFields[i].key, v)) {
        set(c, i, v);  // recorta: un valor fuera de rango en la NVS no llega a la máquina
        n++;
      }
    }
  }
  normalize(c);
  if (found) *found = n;
  return c;
}

bool save(Store &s, const gesture::Config &cIn) {
  gesture::Config c = cIn;
  normalize(c);
  bool ok = true;
  for (int i = 0; i < kCount; i++) ok = s.put(kFields[i].key, get(c, i)) && ok;
  // Un guardado a medias deja una mezcla de valores viejos y nuevos, pero cada uno pasa por set() al cargar.
  return s.put("schema", kSchema) && ok;
}

int32_t clampLightMode(int32_t v) { return v < kLightModeMin ? kLightModeMin : v > kLightModeMax ? kLightModeMax : v; }

int32_t loadLightMode(Store &s) {
  int32_t ver, v;
  if (s.get("schema", ver) && ver == kSchema && s.get("lmode", v)) return clampLightMode(v);
  return kLightModeMin;
}

bool saveLightMode(Store &s, int32_t mode) { return s.put("lmode", clampLightMode(mode)); }

bool loadShock(Store &s) {
  int32_t v;
  return s.get("shock", v) && v == 1;
}

bool saveShock(Store &s, bool on) { return s.put("shock", on ? 1 : 0); }

}  // namespace settings
