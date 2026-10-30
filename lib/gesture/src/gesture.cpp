#include "gesture.h"

namespace gesture {

Config defaults() {
  Config c{};
  // --- tiempos PROVISIONALES: se fijan con los datos medidos del punto 2 (capture de GPIO) ---
  c.debounceUs = 5000;         // 5 ms
  c.doubleClickUs = 300000;    // 300 ms (ventana de doble click; el click simple sale tras esto)
  c.rampIdleUs = 500000;       // 0.5 s sin muescas → fin de la sesión de rampa (emite solo al girar; esto solo
                               // resetea el nivel al piso tras parar; debe superar el tiempo entre muescas al girar)
  c.armHoldUs = 1000000;       // 1 s manteniendo KEY para armar
  c.autoDisarmUs = 30000000;   // 30 s de inactividad → desarmar ("tiempo de bloqueo" en el menú)
  c.autoDisarm = true;         // desactivable desde el menú
  // 0.94 s = cadencia MEDIDA y CONFIRMADA por Daniel (2026-10-20, repetida 3×): gap 700 ms + trama 241 ms. A ese
  // período los 4 pulsos se sienten distintos pero bien encadenados (pausa chica); más rápido se funden/pierden
  // y más lento quedan muy separados. Es el ritmo de la rampa (pulsos discretos; el collar no permite continuo).
  c.minStimGapUs = 940000;     // 0.94 s de inicio-a-inicio entre estímulos (= cadencia de pulso de la rampa)
  c.maxRampUs = 5000000;       // 5 s máximo de rampa continua
  c.keyHoldUs = 400000;        // 400 ms de KEY → pasa de vibrar a tono
  c.maxStimPerMin = 30;        // tope de estímulos por minuto
  // --- niveles conservadores del plan ---
  c.lowPower = 5;      // BAJO
  c.highPower = 15;    // ALTO
  c.minRamp = 5;       // MIN_RAMPA
  c.step = 1;          // PASO
  c.cap = 25;          // TOPE
  c.keyVibrPower = 20; // vibrar (aviso)
  c.keyTonoPower = 10; // tono
  c.wiggleRevs = 4;        // 4 reversas rápidas de la rueda → alterna armado (gesto sin KEY)
  c.wiggleGapUs = 500000;  // cada reversa dentro de 500 ms de la anterior para contar como wiggle
  c.armGraceUs = 800000;   // 0.8 s tras armar sin emitir nada (la cola del wiggle no dispara una rampa/shock)
  c.safeClicks = 2;        // zona de seguridad: 1-2 clicks = solo wiggle; desde el 3.º = rampa (pedido de Daniel)
  return c;
}

Mode effectiveMode(Mode requested, bool shockEnabled) {
  return requested == Mode::Shock && !shockEnabled ? Mode::Vibrar : requested;
}

uint8_t Machine::clampPow(int p) const {
  if (p < 0) p = 0;
  if (p > cfg_.cap) p = cfg_.cap;
  return (uint8_t)p;
}

void Machine::reset(const Config &c, uint32_t now) {
  cfg_ = c;
  // El TOPE manda: recortar la config para que ningún nivel configurado pueda superar cap (invariante ALTO ≤ TOPE).
  if (cfg_.highPower > cfg_.cap) cfg_.highPower = cfg_.cap;
  if (cfg_.lowPower > cfg_.cap) cfg_.lowPower = cfg_.cap;
  if (cfg_.minRamp > cfg_.cap) cfg_.minRamp = cfg_.cap;
  if (cfg_.keyVibrPower > cfg_.cap) cfg_.keyVibrPower = cfg_.cap;
  if (cfg_.keyTonoPower > cfg_.cap) cfg_.keyTonoPower = cfg_.cap;
  if (cfg_.step < 1) cfg_.step = 1;
  armed_ = false;            // SIEMPRE arranca desarmado (H4/H6)
  ramp_ = cfg_.minRamp;
  clickCount_ = 0;
  btnDown_ = keyDown_ = tonoActive_ = doublePending_ = keyLatched_ = rampActive_ = pendingDisarm_ = grace_ = false;
  rDown_ = rRamp_ = false;
  rTaps_ = 0;
  rDownUs_ = rLastUp_ = 0;
  armGraceStart_ = 0;
  revCount_ = 0;
  lastDir_ = 0;
  lastRevUs_ = 0;
  runLen_ = 0;
  lastWheelUs_ = 0;
  rampMode_ = Mode::Shock;
  lastActivity_ = now;
  haveStim_ = false;
  lastStimStart_ = 0;
  startCount_ = startHead_ = 0;
}

void Machine::remoteLost() {
  rDown_ = false;
  rTaps_ = 0;
  if (rRamp_) {
    rRamp_ = false;
    rampActive_ = false;
    ramp_ = cfg_.minRamp;
  }
}

void Machine::push(Action *out, int maxOut, int &n, Act a, Mode m, uint8_t p) {
  if (n < maxOut) {
    out[n].act = a;
    out[n].mode = m;
    out[n].power = p;
  }
  n++;
}

// Un pulso discreto: Stim seguido de Release (H3: toda pulsación cierra con "soltar"). El collar fija la
// duración del pulso, así que cada estímulo es uno de estos; no hay estímulo "sostenido" salvo el tono de KEY.
void Machine::emitPulse(Mode m, uint8_t p, Action *out, int maxOut, int &n) {
  push(out, maxOut, n, Act::Stim, m, clampPow(p));
  push(out, maxOut, n, Act::Release, m, 0);
}

bool Machine::tryStart(uint32_t now) {
  if (grace_) {  // pausa de gracia tras armar: no emitir nada hasta que pase (evita disparo inmediato al armar)
    if ((uint32_t)(now - armGraceStart_) < cfg_.armGraceUs) return false;
    grace_ = false;
  }
  if (haveStim_ && (uint32_t)(now - lastStimStart_) < cfg_.minStimGapUs) return false;  // H8: separación mínima
  uint16_t cnt = 0;                                                                      // H8: tope por minuto
  for (uint8_t i = 0; i < startCount_; i++)
    if ((uint32_t)(now - starts_[i]) <= 60000000u) cnt++;
  if (cnt >= cfg_.maxStimPerMin) return false;
  lastStimStart_ = now;
  haveStim_ = true;
  if (startCount_ < 64) {
    starts_[startCount_++] = now;
  } else {
    starts_[startHead_] = now;
    startHead_ = (startHead_ + 1) & 63;
  }
  return true;
}

int Machine::feed(In in, int32_t data, uint32_t now, Action *out, int maxOut) {
  int n = 0;
  if (armed_) lastActivity_ = now;  // cualquier evento con el equipo armado cuenta como actividad
  bool busy = rampActive_ || tonoActive_;  // una sola cosa en curso (H5)

  switch (in) {
    case In::BtnDown:
      btnDown_ = true;
      if (!armed_ || busy) break;  // H10: desarmado no hace nada · H5: una sola cosa a la vez
      if (clickCount_ == 1 && (uint32_t)(now - lastClickUp_) <= cfg_.doubleClickUs) doublePending_ = true;
      break;

    case In::BtnUp:
      if (!btnDown_) break;
      btnDown_ = false;
      if (!armed_ || busy) break;
      if (doublePending_) {  // H2: doble click confirmado en el segundo flanco → ALTO ya
        doublePending_ = false;
        clickCount_ = 0;
        if (tryStart(now)) emitPulse(Mode::Shock, cfg_.highPower, out, maxOut, n);
      } else {
        clickCount_ = 1;  // posible click simple; se resuelve en tick() al vencer la ventana
        lastClickUp_ = now;
      }
      break;

    case In::Wheel: {
      int8_t dir = data > 0 ? 1 : -1;
      // Zona de seguridad: contar clicks seguidos en este sentido (cambio de sentido o pausa > rampIdle reinicia).
      bool sameRun = runLen_ > 0 && dir == lastDir_ && (uint32_t)(now - lastWheelUs_) < cfg_.rampIdleUs;
      runLen_ = sameRun ? (uint8_t)(runLen_ < 255 ? runLen_ + 1 : 255) : 1;
      lastWheelUs_ = now;
      // Wiggle (reversas rápidas der-izq-der-izq) → alterna ARMADO, funcione armado o no. Confirma con sonido.
      if (lastDir_ != 0 && dir != lastDir_) {
        revCount_ = (uint32_t)(now - lastRevUs_) <= cfg_.wiggleGapUs ? (uint8_t)(revCount_ + 1) : 1;
        lastRevUs_ = now;
        if (revCount_ >= cfg_.wiggleRevs) {
          revCount_ = 0;
          lastDir_ = dir;
          rampActive_ = tonoActive_ = doublePending_ = rRamp_ = false;
          clickCount_ = 0;
          rTaps_ = 0;
          // NO se emite nada al collar: la confirmación de armado es un beep LOCAL del parlante del T-Embed
          // (lo hace el firmware al detectar el cambio de armed()). Aquí solo se cambia el estado.
          if (!armed_) {
            armed_ = true;
            lastActivity_ = now;
            grace_ = true;           // pausa de gracia: la cola del wiggle no debe disparar nada
            armGraceStart_ = now;
          } else {
            pendingDisarm_ = true;  // el desarme se aplica en el próximo tick
          }
          break;
        }
      }
      lastDir_ = dir;
      if (!armed_ || tonoActive_) break;  // DESARMADO: la rueda solo arma (wiggle), no ramplea
      if (!rampActive_ && runLen_ <= cfg_.safeClicks) break;  // dentro de la zona de seguridad: nada sale
      clickCount_ = 0;         // girar cancela un click simple pendiente
      doublePending_ = false;
      if (!rampActive_) { rampActive_ = true; ramp_ = cfg_.minRamp; }  // nueva sesión: arranca en el piso
      rampMode_ = dir > 0 ? Mode::Shock : Mode::Vibrar;  // derecha=shock, izquierda=vibrar (fase: shock→vibrar)
      rampLastDetent_ = now;
      // Un pulso por muesca solo si la cadencia lo permite; si no, la muesca solo mantiene viva la rampa. El nivel
      // sube PASO por PULSO EMITIDO, no por muesca (2026-10-20, Daniel: girando durante el pulso de ~1 s se sumaban
      // ~20 niveles por pulso). Nada fuera de un detent → al dejar de girar, corta (sin cola). H7: tope en TOPE.
      if (tryStart(now)) {
        ramp_ = clampPow(ramp_ + cfg_.step);
        emitPulse(rampMode_, ramp_, out, maxOut, n);
      }
      break;
    }

    case In::RemoteDown: {
      rDown_ = true;
      rDownUs_ = now;
      if (!armed_ || tonoActive_) break;  // H10: desarmado no hace nada
      if (rampActive_) {                  // sesión de rampa en curso: cada tap es una muesca
        rampLastDetent_ = now;
        if (tryStart(now)) {
          ramp_ = clampPow(ramp_ + cfg_.step);
          emitPulse(rampMode_, ramp_, out, maxOut, n);
        }
        break;
      }
      bool inWindow = rTaps_ >= 1 && (uint32_t)(now - rLastUp_) <= cfg_.doubleClickUs;
      rTaps_ = inWindow ? (uint8_t)(rTaps_ + 1) : 1;
      if (rTaps_ >= 3) {  // 3.º tap: arranca la rampa (el BAJO/ALTO pendiente queda cancelado)
        rTaps_ = 0;
        clickCount_ = 0;
        doublePending_ = false;
        rampActive_ = rRamp_ = true;
        ramp_ = cfg_.minRamp;
        rampMode_ = Mode::Shock;
        rampLastDetent_ = now;
        if (tryStart(now)) {
          ramp_ = clampPow(ramp_ + cfg_.step);
          emitPulse(rampMode_, ramp_, out, maxOut, n);
        }
      }
      break;
    }

    case In::RemoteUp:
      if (!rDown_) break;
      rDown_ = false;
      if (rTaps_ >= 1) rLastUp_ = now;  // la ventana para el siguiente tap corre desde que se suelta
      break;

    case In::KeyDown:
      keyDown_ = true;
      keyDownUs_ = now;
      break;

    case In::KeyUp:
      if (!keyDown_) break;
      keyDown_ = false;
      keyLatched_ = false;  // soltar KEY rearma el tono (tras un corte por duración máxima)
      if (!armed_) break;   // la KEY ya NO arma (se arma con el wiggle de la rueda); desarmado no hace nada
      if (tonoActive_) { tonoActive_ = false; break; }  // soltar KEY → deja de repetir el tono
      if (rampActive_) break;                            // ocupado → ignorar
      if ((uint32_t)(now - keyDownUs_) < cfg_.keyHoldUs) {  // toque corto → vibrar de aviso
        if (tryStart(now)) emitPulse(Mode::Vibrar, cfg_.keyVibrPower, out, maxOut, n);
      }
      break;
  }
  return n;
}

int Machine::tick(uint32_t now, Action *out, int maxOut) {
  int n = 0;

  // Desarme diferido: el buzz de confirmación ya salió (aún armado); ahora sí se baja el armado.
  if (pendingDisarm_) {
    pendingDisarm_ = false;
    armed_ = false;
    rampActive_ = tonoActive_ = doublePending_ = rRamp_ = false;
    clickCount_ = 0;
    rTaps_ = 0;
  }

  // Click simple: la ventana de doble venció sin segundo click y sin botón apretado → BAJO.
  if (armed_ && clickCount_ == 1 && !doublePending_ && !btnDown_ && !rampActive_ && !tonoActive_ &&
      (uint32_t)(now - lastClickUp_) > cfg_.doubleClickUs) {
    clickCount_ = 0;
    if (tryStart(now)) emitPulse(Mode::Shock, cfg_.lowPower, out, maxOut, n);
  }

  // Botón remoto atascado (el RemoteUp se perdió por el aire): tras maxRampUs se da por soltado, sin emitir.
  if (rDown_ && (uint32_t)(now - rDownUs_) >= cfg_.maxRampUs) {
    rDown_ = false;
    rTaps_ = 0;
  }
  // Taps remotos: venció la ventana con el botón suelto → 1 tap = BAJO, 2 taps = ALTO (3+ ya arrancó la rampa).
  if (armed_ && rTaps_ >= 1 && !rDown_ && !rampActive_ && !tonoActive_ &&
      (uint32_t)(now - rLastUp_) > cfg_.doubleClickUs) {
    uint8_t taps = rTaps_;
    rTaps_ = 0;
    if (tryStart(now)) emitPulse(Mode::Shock, taps == 1 ? cfg_.lowPower : cfg_.highPower, out, maxOut, n);
  }
  if (!armed_) rTaps_ = 0;

  // Rampa: aquí SOLO se gestiona el fin. Emitir es exclusivo del detent (feed), así que al dejar de girar no
  // sale nada más. Sin muescas por rampIdle (H3) → fin de sesión y nivel al piso, listo para la próxima.
  // La remota por taps espera más: un tap por pulso (minStimGap) + holgura, si no cada tap reiniciaría el nivel.
  uint32_t idle = rRamp_ ? cfg_.minStimGapUs + cfg_.rampIdleUs : cfg_.rampIdleUs;
  if (rampActive_ && (uint32_t)(now - rampLastDetent_) >= idle) {
    rampActive_ = false;
    rRamp_ = false;
    ramp_ = cfg_.minRamp;
  }

  // KEY mantenida → tono REPETIDO: un pulso de tono cada cadencia mientras se mantiene (el collar no sostiene,
  // hay que re-disparar). Tope de duración (H1-key): KEY trabado no suena sin fin → latchea hasta soltar.
  if (armed_ && keyDown_ && !keyLatched_ && !rampActive_ && (uint32_t)(now - keyDownUs_) >= cfg_.keyHoldUs) {
    if (!tonoActive_) { tonoActive_ = true; tonoStart_ = now; }
    if ((uint32_t)(now - tonoStart_) >= cfg_.maxRampUs) {
      keyLatched_ = true;
      tonoActive_ = false;
    } else if (tryStart(now)) {
      emitPulse(Mode::Tono, cfg_.keyTonoPower, out, maxOut, n);
    }
  }

  // Auto-desarme por inactividad (H6), solo si está activado y no hay nada en curso ni KEY apretada.
  if (cfg_.autoDisarm && armed_ && !rampActive_ && !tonoActive_ && !keyDown_ &&
      (uint32_t)(now - lastActivity_) >= cfg_.autoDisarmUs) {
    armed_ = false;
    clickCount_ = 0;
    doublePending_ = false;
    rTaps_ = 0;
  }
  return n;
}

}  // namespace gesture
