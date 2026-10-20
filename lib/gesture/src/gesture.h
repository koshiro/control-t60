// Máquina de gestos y armado del control T60 — lógica PURA, sin Arduino, probada en el host.
// Decide qué estímulo sale a partir de eventos con timestamp (botón del encoder, rueda, KEY) y del
// estado de armado. No toca el CC1101 ni el RMT: solo produce acciones (Stim/Release con modo y potencia)
// que la tarea de TX del firmware convierte en tramas. Así las invariantes de seguridad se verifican en
// `pio test -e native` sin hardware. Plan y tabla de peligros: references/control-firmware.md (fase 2).
//
// Mapeo de gestos (el firmware, en fase de prueba, sustituye Shock→Vibrar porque ALLOW_SHOCK=0):
//   click simple        -> Stim(Shock, BAJO)      + Release
//   doble click         -> Stim(Shock, ALTO)      + Release
//   rueda, giro sostenido -> rampa de PULSOS discretos (Stim+Release) subiendo de nivel, al ritmo del collar.
//                          Sentido: derecha (>0) = Shock, izquierda (<0) = Vibrar (ambos SUBEN). El collar NO
//                          sigue el cambio de potencia dentro de una pulsación (verificado 2026-10-20).
//   rueda, wiggle (reversas rápidas der-izq-der-izq) -> alterna ARMADO/desarmado (la KEY es incómoda). Confirma
//                          con sonido sin mirar: armar = beep (Tono), desarmar = buzz (Vibrar).
//   KEY toque corto     -> Stim(Vibrar, keyVibr)  + Release
//   KEY mantenida       -> pulsos de Tono REPETIDOS a la cadencia mientras se mantiene (el collar no sostiene;
//                          re-disparar es la única forma de que suene repetido). Cada pulso = Stim + Release.
// Nada sale si no está ARMADO. Armar = mantener KEY >= armHoldUs estando DESARMADO. Desarme por inactividad.
#pragma once
#include <stddef.h>
#include <stdint.h>

namespace gesture {

enum class In : uint8_t { BtnDown, BtnUp, Wheel, KeyDown, KeyUp };  // Wheel lleva data = +1/-1 (sentido)
enum class Mode : uint8_t { None, Shock, Vibrar, Tono, Luz };
enum class Act : uint8_t { None, Stim, Release };

struct Action {
  Act act;
  Mode mode;
  uint8_t power;
};

// Todos los tiempos en µs (diferencias wrap-safe en uint32_t para intervalos < ~71 min).
struct Config {
  uint32_t debounceUs;     // rebote de flancos de botón/KEY (lo aplica el firmware antes de feed; aquí informativo)
  uint32_t doubleClickUs;  // separación máxima click→click para contar doble
  uint32_t rampIdleUs;     // sin detents por este tiempo → cerrar rampa (soltar)
  uint32_t armHoldUs;      // KEY mantenida (desarmado) para armar
  uint32_t autoDisarmUs;   // inactividad (armado) → desarmar
  uint32_t minStimGapUs;   // separación mínima entre INICIOS de estímulo
  uint32_t maxRampUs;      // duración máxima de una rampa → forzar soltar
  uint32_t keyHoldUs;      // KEY mantenida (armado) para pasar de vibrar a tono
  uint16_t maxStimPerMin;  // tope de inicios de estímulo en 60 s móviles
  uint8_t lowPower;        // BAJO (click simple)
  uint8_t highPower;       // ALTO (doble click)
  uint8_t minRamp;         // MIN_RAMPA (piso de la rampa)
  uint8_t step;            // PASO por detent
  uint8_t cap;             // TOPE (máximo duro; reset() recorta todo a <= cap)
  uint8_t keyVibrPower;    // nivel de vibrar del toque de KEY
  uint8_t keyTonoPower;    // nivel de tono de KEY mantenida
  uint8_t wiggleRevs;      // reversas de giro rápidas para alternar armado (gesto de la rueda)
  uint32_t wiggleGapUs;    // separación máxima entre reversas para contarlas como un wiggle
  uint32_t armGraceUs;     // tras armar, periodo en que se ignora todo estímulo (para que la cola del wiggle no dispare)
  bool autoDisarm;         // desarme automático por inactividad (autoDisarmUs); false = queda armado hasta el wiggle
  uint8_t safeClicks;      // ZONA DE SEGURIDAD: armado, los primeros N clicks seguidos en un sentido no emiten (solo
                           // wiggle); la rampa empieza en el click N+1. Reinicia al cambiar de sentido o tras rampIdle.
};

Config defaults();

// Modo que realmente sale al collar: con el interruptor de shock apagado (o ALLOW_SHOCK=0) el shock se sustituye
// por vibrar; los demás modos pasan tal cual.
Mode effectiveMode(Mode requested, bool shockEnabled);  // valores conservadores del plan; los de tiempo son PROVISIONALES hasta el punto 2 (medición)

class Machine {
 public:
  void reset(const Config &c, uint32_t nowUs);
  // Procesa un evento. Devuelve cuántas acciones escribió en `out` (puede ser 0). `data` solo lo usa Wheel.
  int feed(In in, int32_t data, uint32_t nowUs, Action *out, int maxOut);
  // Vence timeouts (click simple, tono, rampa, auto-desarme). Llamar seguido. Devuelve nº de acciones.
  int tick(uint32_t nowUs, Action *out, int maxOut);

  bool armed() const { return armed_; }
  bool pressActive() const { return rampActive_ || tonoActive_; }
  uint8_t rampLevel() const { return ramp_; }
  const Config &config() const { return cfg_; }

 private:
  bool tryStart(uint32_t now);  // puerta de ritmo (minStimGap + maxStimPerMin); registra el inicio si pasa
  void push(Action *out, int maxOut, int &n, Act a, Mode m, uint8_t p);
  void emitPulse(Mode m, uint8_t p, Action *out, int maxOut, int &n);  // un pulso discreto = Stim + Release
  uint8_t clampPow(int p) const;

  Config cfg_{};
  bool armed_ = false, btnDown_ = false, keyDown_ = false, tonoActive_ = false, doublePending_ = false,
       haveStim_ = false, keyLatched_ = false, rampActive_ = false, pendingDisarm_ = false, grace_ = false;
  uint32_t armGraceStart_ = 0;
  uint8_t clickCount_ = 0, ramp_ = 0, revCount_ = 0;
  int8_t lastDir_ = 0;
  uint8_t runLen_ = 0;        // clicks seguidos en el mismo sentido (zona de seguridad)
  uint32_t lastWheelUs_ = 0;
  Mode rampMode_ = Mode::Shock;
  uint32_t lastClickUp_ = 0, keyDownUs_ = 0, rampLastDetent_ = 0, tonoStart_ = 0, lastRevUs_ = 0, lastActivity_ = 0,
           lastStimStart_ = 0;
  uint32_t starts_[64] = {};  // ring de inicios para el tope por minuto
  uint8_t startCount_ = 0, startHead_ = 0;
};

}  // namespace gesture
