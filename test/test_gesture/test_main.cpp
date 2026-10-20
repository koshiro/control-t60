// Pruebas en el host de la lógica pura de gestos/armado: pio test -e native.
//   1. Líneas de tiempo en tablas (cada fila cubre una fila de la tabla de peligros).
//   2. Verificador de invariantes de seguridad, independiente de la máquina.
//   3. Fuzz: miles de secuencias aleatorias de semilla fija → el verificador no encuentra violaciones.
//   4. Mutación: trazas corrompidas a propósito → el verificador SÍ las detecta (prueba que tiene dientes).
#include <stdio.h>
#include <string.h>
#include <unity.h>

#include <random>
#include <vector>

#include "gesture.h"

using namespace gesture;

void setUp() {}
void tearDown() {}

// ---- registro de lo que produjo la máquina: acción + instante + si estaba armada en ese momento ----
struct Rec {
  uint32_t t;
  Action a;
  bool armed;
};

// Corre la máquina sobre un guion de eventos (t, in, data), intercalando ticks cada `tickUs`, y registra
// cada acción emitida. Al final drena ticks para cerrar pulsaciones abiertas.
struct Step {
  uint32_t t;
  In in;
  int32_t data;
};
static void run(Machine &m, const std::vector<Step> &steps, std::vector<Rec> &rec, uint32_t startT = 0,
                uint32_t tickUs = 20000) {
  Action out[8];
  uint32_t t = startT;
  size_t i = 0;
  auto record = [&](int nn, uint32_t now) {
    for (int k = 0; k < nn && k < 8; k++) rec.push_back({now, out[k], m.armed()});
  };
  uint32_t end = steps.empty() ? 0 : steps.back().t;
  end += 10000000;  // 10 s extra de ticks para vencer timeouts largos (auto-desarme, maxRamp)
  for (; t <= end; t += tickUs) {
    while (i < steps.size() && steps[i].t <= t) {
      record(m.feed(steps[i].in, steps[i].data, steps[i].t, out, 8), steps[i].t);
      i++;
    }
    record(m.tick(t, out, 8), t);
  }
}

// ---- verificador de invariantes (independiente de Machine): recorre el registro de acciones ----
// Reconstruye "inicios de pulsación" = Stim cuyo estado previo era cerrado (tras Release o al comienzo).
struct Violation {
  const char *rule;
  int index;
};
static Violation check(const std::vector<Rec> &rec, const Config &cfg) {
  bool open = false;
  bool haveStart = false;
  uint32_t lastStart = 0;
  std::vector<uint32_t> starts;
  for (size_t k = 0; k < rec.size(); k++) {
    const Action &a = rec[k].a;
    if (a.act == Act::Stim) {
      if (a.power > cfg.cap) return {"potencia > TOPE", (int)k};                 // inv: potencia ≤ TOPE
      if (!rec[k].armed) return {"TX estando desarmado", (int)k};               // inv: nunca TX desarmado
      bool start = !open;
      open = true;
      if (start) {
        if (haveStart && (uint32_t)(rec[k].t - lastStart) < cfg.minStimGapUs)   // inv: separación mínima
          return {"estímulos demasiado juntos", (int)k};
        uint16_t cnt = 1;
        for (uint32_t s : starts)
          if ((uint32_t)(rec[k].t - s) <= 60000000u) cnt++;
        if (cnt > cfg.maxStimPerMin) return {"más estímulos por minuto que el tope", (int)k};  // inv
        starts.push_back(rec[k].t);
        lastStart = rec[k].t;
        haveStart = true;
      }
    } else if (a.act == Act::Release) {
      if (!open) return {"soltar sin pulsación abierta", (int)k};
      open = false;
    }
  }
  if (open) return {"pulsación quedó sin soltar", (int)rec.size()};  // inv: siempre "soltar"
  return {nullptr, -1};
}

// ================================ 1. líneas de tiempo en tablas ================================

// Arma la máquina con un wiggle de la rueda (reversas rápidas der-izq-der-izq), SIN grabar (es setup).
// Devuelve un tiempo de inicio para run() posterior a este gesto. Deja la máquina ARMADA.
static uint32_t armDirect(Machine &m) {
  Action o[8];
  uint32_t t = 100000;
  for (int i = 0; i < 5; i++) {  // 5 detents alternados → 4 reversas = wiggleRevs → arma en el 5º
    m.feed(In::Wheel, (i & 1) ? -1 : 1, t, o, 8);
    t += 150000;
  }
  return 1900000;  // run() arranca aquí (después del wiggle, antes de los eventos del test en ≥2.0 s)
}

// El wiggle arma, y NO manda nada al collar (la confirmación es un beep local del T-Embed). (gesto sin KEY)
static void test_arm_via_wiggle() {
  Machine m;
  m.reset(defaults(), 0);
  std::vector<Rec> r;
  Action o[8];
  uint32_t t = 100000;
  for (int i = 0; i < 5; i++) {
    int nn = m.feed(In::Wheel, (i & 1) ? -1 : 1, t, o, 8);
    for (int k = 0; k < nn; k++) r.push_back({t, o[k], m.armed()});
    t += 150000;
  }
  TEST_ASSERT_TRUE_MESSAGE(m.armed(), "el wiggle debe armar");
  int stims = 0;
  for (auto &x : r) stims += (x.a.act == Act::Stim);
  TEST_ASSERT_EQUAL_INT_MESSAGE(0, stims, "armar NO debe transmitir al collar (beep es local)");
}

// Pocas reversas (lentas o pocas) NO arman. (evita armado accidental)
static void test_few_reversals_dont_arm() {
  Machine m;
  m.reset(defaults(), 0);
  std::vector<Rec> r;
  // solo 2 reversas → no alcanza wiggleRevs
  run(m, {{100000, In::Wheel, +1}, {250000, In::Wheel, -1}, {400000, In::Wheel, +1}}, r);
  TEST_ASSERT_FALSE_MESSAGE(m.armed(), "2 reversas no deben armar");
  TEST_ASSERT_EQUAL_INT_MESSAGE(0, (int)r.size(), "desarmado no emite nada");
}

// Tras armar hay una pausa de gracia: girar inmediatamente (cola del wiggle) NO emite estímulo.
static void test_arm_grace_suppresses_immediate() {
  Machine m;
  m.reset(defaults(), 0);
  Action o[8];
  uint32_t t = 100000;
  for (int i = 0; i < 5; i++) { m.feed(In::Wheel, (i & 1) ? -1 : 1, t, o, 8); t += 150000; }  // arma en ~700 ms
  std::vector<Rec> r;
  for (int i = 0; i < 4; i++) {  // seguir girando dentro de la gracia (0.8 s)
    int nn = m.feed(In::Wheel, +1, t, o, 8);
    for (int k = 0; k < nn; k++) r.push_back({t, o[k], m.armed()});
    t += 100000;
  }
  int stims = 0;
  for (auto &x : r) stims += x.a.act == Act::Stim;
  TEST_ASSERT_TRUE(m.armed());
  TEST_ASSERT_EQUAL_INT_MESSAGE(0, stims, "durante la gracia tras armar no debe emitir");
}

// Desarmado: un click no produce nada. (H10: nunca TX desarmado)
static void test_click_while_disarmed_is_silent() {
  Machine m;
  m.reset(defaults(), 0);
  std::vector<Rec> r;
  run(m, {{100000, In::BtnDown, 0}, {150000, In::BtnUp, 0}}, r);
  TEST_ASSERT_EQUAL_INT(0, (int)r.size());
}

// Click simple armado → Shock(BAJO) + Release, tras vencer la ventana de doble. (H2)
static void test_single_click_low() {
  Machine m;
  m.reset(defaults(), 0);
  std::vector<Step> s;
  uint32_t t0 = armDirect(m);
  s.push_back({2000000, In::BtnDown, 0});
  s.push_back({2050000, In::BtnUp, 0});
  std::vector<Rec> r;
  run(m, s, r, t0);
  int stim = -1, rel = -1;
  for (size_t k = 0; k < r.size(); k++) {
    if (r[k].a.act == Act::Stim && stim < 0) stim = (int)k;
    if (r[k].a.act == Act::Release && stim >= 0 && rel < 0) rel = (int)k;
  }
  TEST_ASSERT_TRUE_MESSAGE(stim >= 0, "falta el estímulo del click");
  TEST_ASSERT_EQUAL_UINT8(defaults().lowPower, r[stim].a.power);
  TEST_ASSERT_EQUAL_INT((int)Mode::Shock, (int)r[stim].a.mode);
  TEST_ASSERT_TRUE_MESSAGE(rel > stim, "el click debe cerrar con soltar");
  // sale después de la ventana de doble click
  TEST_ASSERT_TRUE(r[stim].t >= 2050000 + defaults().doubleClickUs);
}

// Doble click → Shock(ALTO) + Release, y NO sale un BAJO. (H2)
static void test_double_click_high_no_low() {
  Machine m;
  m.reset(defaults(), 0);
  std::vector<Step> s;
  uint32_t t0 = armDirect(m);
  s.push_back({2000000, In::BtnDown, 0});
  s.push_back({2050000, In::BtnUp, 0});
  s.push_back({2200000, In::BtnDown, 0});  // 2º dentro de 300 ms
  s.push_back({2250000, In::BtnUp, 0});
  std::vector<Rec> r;
  run(m, s, r, t0);
  int stims = 0;
  uint8_t pw = 0;
  for (auto &x : r)
    if (x.a.act == Act::Stim) {
      stims++;
      pw = x.a.power;
    }
  TEST_ASSERT_EQUAL_INT_MESSAGE(1, stims, "el doble click debe dar un solo estímulo (ALTO), no también BAJO");
  TEST_ASSERT_EQUAL_UINT8(defaults().highPower, pw);
}

// Rampa = pulsos discretos (el collar no sigue la potencia dentro de un botón). Detents espaciados > minStimGap
// → un pulso por nivel 6,7,8, cada uno Stim+Release; al quedar quieta el nivel vuelve a 5. (H3)
static void test_ramp_discrete_pulses_spaced() {
  Machine m;
  Config c = defaults();
  c.minStimGapUs = 250000;   // esta prueba valida el MECANISMO (un pulso por detent); cadencia chica a propósito
  c.rampIdleUs = 500000;
  c.safeClicks = 0;          // sin zona de seguridad (la cubre test_safety_zone_*)
  m.reset(c, 0);
  std::vector<Step> s;
  uint32_t t0 = armDirect(m);
  s.push_back({2000000, In::Wheel, +1});  // detents a 300 ms: < rampIdle (sigue la sesión), > minStimGap (cada uno emite)
  s.push_back({2300000, In::Wheel, +1});
  s.push_back({2600000, In::Wheel, +1});
  std::vector<Rec> r;
  run(m, s, r, t0);
  std::vector<uint8_t> levels;
  int rel = 0;
  for (auto &x : r) {
    if (x.a.act == Act::Stim) levels.push_back(x.a.power);
    if (x.a.act == Act::Release) rel++;
  }
  TEST_ASSERT_EQUAL_INT(3, (int)levels.size());
  TEST_ASSERT_EQUAL_UINT8(6, levels[0]);
  TEST_ASSERT_EQUAL_UINT8(7, levels[1]);
  TEST_ASSERT_EQUAL_UINT8(8, levels[2]);
  TEST_ASSERT_EQUAL_INT_MESSAGE(3, rel, "cada pulso de rampa cierra con soltar");
  TEST_ASSERT_EQUAL_UINT8_MESSAGE(c.minRamp, m.rampLevel(), "la rampa vuelve al piso");
  TEST_ASSERT_FALSE(m.pressActive());
}

// Giro rápido: los detents intermedios se DESCARTAN por el ritmo (no se encolan) y, al dejar de girar, NO
// sale ningún pulso más (lo que pidió Daniel). (H8 + "cortar al soltar la rueda")
static void test_ramp_fast_spin_drops_and_stops() {
  Machine m;
  Config c = defaults();
  m.reset(c, 0);
  std::vector<Step> s;
  uint32_t t0 = armDirect(m);
  uint32_t lastDetent = 0;
  for (int i = 0; i < 10; i++) {
    uint32_t t = 2000000 + (uint32_t)i * 50000;  // 10 detents a 50 ms (mucho más rápido que la cadencia)
    s.push_back({t, In::Wheel, +1});
    lastDetent = t;
  }
  std::vector<Rec> r;
  run(m, s, r, t0);
  int stims = 0;
  uint32_t lastStim = 0;
  uint8_t mx = 0;
  for (auto &x : r)
    if (x.a.act == Act::Stim) {
      stims++;
      lastStim = x.t;
      if (x.a.power > mx) mx = x.a.power;
    }
  TEST_ASSERT_TRUE_MESSAGE(stims >= 1, "al menos el primer pulso sale");
  TEST_ASSERT_TRUE_MESSAGE(mx <= c.cap, "nunca supera TOPE");
  TEST_ASSERT_TRUE_MESSAGE(lastStim <= lastDetent, "no se emite ningún pulso después de dejar de girar");
  TEST_ASSERT_FALSE(m.pressActive());
  TEST_ASSERT_EQUAL_INT(-1, check(r, c).index);
}

// Rampa con muchísimos pulsos → nunca supera el TOPE. (H7)
static void test_ramp_caps_at_top() {
  Machine m;
  Config c = defaults();
  c.minStimGapUs = 50000;  // un pulso por detent (el nivel sube por pulso) para llegar al TOPE
  m.reset(c, 0);
  std::vector<Step> s;
  uint32_t t0 = armDirect(m);
  for (int i = 0; i < 100; i++) s.push_back({2000000 + (uint32_t)i * 50000, In::Wheel, +1});
  std::vector<Rec> r;
  run(m, s, r, t0);
  uint8_t mx = 0;
  for (auto &x : r)
    if (x.a.act == Act::Stim) mx = x.a.power > mx ? x.a.power : mx;
  TEST_ASSERT_EQUAL_UINT8_MESSAGE(c.cap, mx, "la rampa debe toparse en TOPE");
}

// Giro sostenido → el nº de pulsos está acotado por maxStimPerMin en la ventana de 60 s. (H8/H9)
static void test_ramp_rate_capped_per_minute() {
  Machine m;
  Config c = defaults();
  c.maxStimPerMin = 5;
  c.minStimGapUs = 100000;  // 100 ms
  m.reset(c, 0);
  std::vector<Step> s;
  uint32_t t0 = armDirect(m);
  for (int i = 0; i < 60; i++) s.push_back({2000000 + (uint32_t)i * 100000, In::Wheel, +1});  // 6 s de giro
  std::vector<Rec> r;
  run(m, s, r, t0);
  int stims = 0;
  for (auto &x : r) stims += x.a.act == Act::Stim;
  TEST_ASSERT_TRUE_MESSAGE(stims <= 5, "nunca más que maxStimPerMin en 60 s, aunque gire sin parar");
  TEST_ASSERT_EQUAL_INT(-1, check(r, c).index);
}

// Dos clicks más juntos que minStimGap → el segundo se suprime. (H8)
static void test_rate_limit_suppresses_second() {
  Machine m;
  Config c = defaults();
  c.minStimGapUs = 500000;  // fija el piso para esta prueba (independiente del default)
  m.reset(c, 0);
  std::vector<Step> s;
  uint32_t t0 = armDirect(m);
  // click 1
  s.push_back({2000000, In::BtnDown, 0});
  s.push_back({2050000, In::BtnUp, 0});
  // click 2, apenas 100 ms después de que el 1 resuelva (dentro de minStimGap 500 ms)
  uint32_t t2 = 2050000 + defaults().doubleClickUs + 100000;
  s.push_back({t2, In::BtnDown, 0});
  s.push_back({t2 + 50000, In::BtnUp, 0});
  std::vector<Rec> r;
  run(m, s, r, t0);
  int stims = 0;
  for (auto &x : r) stims += x.a.act == Act::Stim;
  TEST_ASSERT_EQUAL_INT_MESSAGE(1, stims, "el 2º click dentro de minStimGap debe suprimirse");
}

// KEY mantenida → el tono se REPITE a la cadencia; pasado maxRampUs latchea y una KEY trabada deja de sonar. (H1 KEY)
static void test_key_tono_repeats_and_caps() {
  Machine m;
  Config c = defaults();
  c.maxRampUs = 2000000;  // 2 s de tono máximo
  m.reset(c, 0);
  std::vector<Step> s;
  uint32_t t0 = armDirect(m);
  s.push_back({2000000, In::KeyDown, 0});  // mantenida, NUNCA se suelta (botón trabado)
  std::vector<Rec> r;
  run(m, s, r, t0);
  int tono = 0;
  for (auto &x : r) tono += (x.a.act == Act::Stim && x.a.mode == Mode::Tono);
  TEST_ASSERT_TRUE_MESSAGE(tono >= 2, "el tono debe REPETIRSE mientras se mantiene KEY");
  TEST_ASSERT_FALSE_MESSAGE(m.pressActive(), "tras el tope de duración, KEY trabada deja de sonar");
  TEST_ASSERT_EQUAL_INT(-1, check(r, c).index);
}

// Auto-desarme por inactividad. (H6)
static void test_auto_disarm() {
  Machine m;
  Config c = defaults();
  c.autoDisarmUs = 2000000;  // 2 s
  m.reset(c, 0);
  std::vector<Step> s;
  uint32_t t0 = armDirect(m);
  std::vector<Rec> r;
  run(m, s, r, t0);  // run drena ~10 s de ticks → pasa el auto-desarme
  TEST_ASSERT_FALSE_MESSAGE(m.armed(), "debe desarmarse por inactividad");
}

// Auto-desarme desactivado (toggle del menú): queda armado pese a la inactividad; el wiggle sigue desarmando.
static void test_auto_disarm_can_be_disabled() {
  Machine m;
  Config c = defaults();
  c.autoDisarmUs = 2000000;
  c.autoDisarm = false;
  m.reset(c, 0);
  std::vector<Step> s;
  uint32_t t0 = armDirect(m);
  std::vector<Rec> r;
  run(m, s, r, t0);  // ~10 s de ticks sin tocar nada
  TEST_ASSERT_TRUE_MESSAGE(m.armed(), "sin auto-desarme debe seguir armado");
  for (int i = 0; i < 5; i++) s.push_back({13000000u + (uint32_t)i * 150000u, In::Wheel, (i & 1) ? -1 : 1});
  run(m, s, r, 12500000);
  TEST_ASSERT_FALSE_MESSAGE(m.armed(), "el wiggle desarma igual");
  TEST_ASSERT_EQUAL_INT(-1, check(r, c).index);
}

// Arranca desarmado siempre. (H4)
static void test_boots_disarmed() {
  Machine m;
  m.reset(defaults(), 123456);
  TEST_ASSERT_FALSE(m.armed());
  TEST_ASSERT_FALSE(m.pressActive());
}

// ================================ 2+3. invariantes + fuzz ================================

static void test_timelines_satisfy_invariants() {
  // Reaprovecha varios guiones y verifica que ninguno viola invariantes.
  const Config c = defaults();
  struct Case {
    const char *name;
    std::vector<Step> steps;
  };
  std::vector<Case> cases;
  {
    std::vector<Step> s;
    s.push_back({2000000, In::BtnDown, 0});
    s.push_back({2050000, In::BtnUp, 0});
    cases.push_back({"click", s});
  }
  {
    std::vector<Step> s;
    for (int i = 0; i < 100; i++) s.push_back({2000000 + (uint32_t)i * 50000, In::Wheel, +1});
    cases.push_back({"rampa larga", s});
  }
  for (auto &cs : cases) {
    Machine m;
    m.reset(c, 0);
    uint32_t t0 = armDirect(m);
    std::vector<Rec> r;
    run(m, cs.steps, r, t0);
    Violation v = check(r, c);
    TEST_ASSERT_EQUAL_INT_MESSAGE(-1, v.index, cs.name);
  }
}

// Miles de secuencias aleatorias de semilla fija → sin violaciones de invariantes.
static void test_fuzz_random_sequences() {
  std::mt19937 rng(0x7460C0);
  const int N = 4000;
  for (int it = 0; it < N; it++) {
    Config c = defaults();
    // variar algunos parámetros, siempre con TOPE dominante
    c.cap = 5 + rng() % 30;
    c.highPower = rng() % 60;  // reset() lo recortará a cap → sirve para probar el recorte
    c.lowPower = rng() % 60;
    c.step = 1 + rng() % 4;
    c.minRamp = rng() % 40;
    c.maxStimPerMin = 3 + rng() % 40;
    Machine m;
    m.reset(c, 0);
    Action out[8];
    std::vector<Rec> rec;
    uint32_t t = 0;
    int nev = 20 + rng() % 60;
    for (int e = 0; e < nev; e++) {
      t += 1000 + rng() % 500000;  // 1 µs..0.5 s entre pasos
      // a veces un tick, a veces un evento
      if (rng() % 3 == 0) {
        int nn = m.tick(t, out, 8);
        for (int k = 0; k < nn; k++) rec.push_back({t, out[k], m.armed()});
      }
      In in = (In)(rng() % 5);
      int32_t data = (rng() & 1) ? +1 : -1;
      int nn = m.feed(in, data, t, out, 8);
      for (int k = 0; k < nn; k++) rec.push_back({t, out[k], m.armed()});
    }
    // drenar ticks para cerrar cualquier pulsación abierta
    for (int e = 0; e < 20; e++) {
      t += 1000000;
      int nn = m.tick(t, out, 8);
      for (int k = 0; k < nn; k++) rec.push_back({t, out[k], m.armed()});
    }
    Violation v = check(rec, m.config());
    if (v.index >= 0) {
      char msg[128];
      snprintf(msg, sizeof msg, "iter %d: invariante violada: %s (accion %d)", it, v.rule, v.index);
      TEST_FAIL_MESSAGE(msg);
    }
  }
}

// ================================ 4. mutación ================================
// Toma una traza válida y le inyecta cada tipo de falla; el verificador DEBE detectarlas todas.
static void test_mutation_checker_has_teeth() {
  Config c = defaults();
  c.minStimGapUs = 500000;  // valida el verificador; gap chico para que la base tenga dos estímulos
  Machine m;
  m.reset(c, 0);
  std::vector<Step> s;
  uint32_t t0 = armDirect(m);
  s.push_back({2000000, In::BtnDown, 0});
  s.push_back({2050000, In::BtnUp, 0});                 // click 1
  s.push_back({3000000, In::BtnDown, 0});
  s.push_back({3050000, In::BtnUp, 0});                 // click 2 (separado > minStimGap)
  std::vector<Rec> base;
  run(m, s, base, t0);
  TEST_ASSERT_EQUAL_INT_MESSAGE(-1, check(base, c).index, "la traza base debe ser válida");

  auto firstStim = [&](std::vector<Rec> &r) {
    for (size_t k = 0; k < r.size(); k++)
      if (r[k].a.act == Act::Stim) return (int)k;
    return -1;
  };

  // M1: potencia por encima del TOPE
  {
    std::vector<Rec> r = base;
    r[firstStim(r)].a.power = c.cap + 1;
    TEST_ASSERT_TRUE_MESSAGE(check(r, c).index >= 0, "M1 potencia>TOPE no detectada");
  }
  // M2: estímulo estando desarmado
  {
    std::vector<Rec> r = base;
    r[firstStim(r)].armed = false;
    TEST_ASSERT_TRUE_MESSAGE(check(r, c).index >= 0, "M2 TX desarmado no detectado");
  }
  // M3: falta un "soltar" (se borra el último Release) → pulsación sin cerrar
  {
    std::vector<Rec> r = base;
    for (int k = (int)r.size() - 1; k >= 0; k--)
      if (r[k].a.act == Act::Release) {
        r.erase(r.begin() + k);
        break;
      }
    TEST_ASSERT_TRUE_MESSAGE(check(r, c).index >= 0, "M3 falta de soltar no detectada");
  }
  // M4: dos inicios demasiado juntos (adelantar el 2º click al mismo instante del 1º)
  {
    std::vector<Rec> r = base;
    int s1 = firstStim(r);
    int s2 = -1;
    for (size_t k = s1 + 1; k < r.size(); k++)
      if (r[k].a.act == Act::Stim) {
        s2 = (int)k;
        break;
      }
    TEST_ASSERT_TRUE_MESSAGE(s2 > 0, "la base debe tener dos estímulos");
    r[s2].t = r[s1].t + 1000;  // 1 ms < minStimGap
    TEST_ASSERT_TRUE_MESSAGE(check(r, c).index >= 0, "M4 estímulos demasiado juntos no detectados");
  }
}

// Menú abierto con KEY+botón (DESARMADO): el firmware deja de pasarle eventos a la máquina, así que los KeyUp/BtnUp
// de la combinación nunca llegan. Al salir del menú se hace reset() → no queda KEY "pegada" que al re-armar dispare
// tono. El caso sin reset() demuestra que el peligro es real (la prueba tiene dientes).
// Zona de seguridad (armado): 1-2 clicks en un sentido no emiten nada; el 3.º empieza la rampa.
static int stimsAfterClicks(int n, uint32_t spacingUs, std::vector<Rec> *out = nullptr) {
  Machine m;
  Config c = defaults();
  c.minStimGapUs = 250000;
  m.reset(c, 0);
  std::vector<Step> s;
  uint32_t t0 = armDirect(m);
  for (int i = 0; i < n; i++) s.push_back({2000000u + (uint32_t)i * spacingUs, In::Wheel, +1});
  std::vector<Rec> r;
  run(m, s, r, t0);
  int stims = 0;
  for (auto &x : r) stims += x.a.act == Act::Stim;
  TEST_ASSERT_EQUAL_INT(-1, check(r, c).index);
  if (out) *out = r;
  return stims;
}
static void test_safety_zone_two_clicks_silent() {
  TEST_ASSERT_EQUAL_INT_MESSAGE(0, stimsAfterClicks(1, 300000), "1 click: zona de seguridad");
  TEST_ASSERT_EQUAL_INT_MESSAGE(0, stimsAfterClicks(2, 300000), "2 clicks: zona de seguridad");
}
static void test_safety_zone_third_click_starts_ramp() {
  std::vector<Rec> r;
  TEST_ASSERT_EQUAL_INT_MESSAGE(1, stimsAfterClicks(3, 300000, &r), "el 3.er click es rampa");
  for (auto &x : r)
    if (x.a.act == Act::Stim) TEST_ASSERT_EQUAL_UINT8(defaults().minRamp + defaults().step, x.a.power);
  TEST_ASSERT_EQUAL_INT_MESSAGE(2, stimsAfterClicks(4, 300000), "3.º y 4.º emiten (cadencia 250 ms)");
}
// Clicks sueltos con pausa > rampIdle no se acumulan: cada uno vuelve a empezar la zona.
static void test_safety_zone_resets_after_pause() {
  TEST_ASSERT_EQUAL_INT(0, stimsAfterClicks(4, 600000));  // 600 ms > rampIdle 500 ms
}
// Vaivén de 2 clicks por lado estando armado: nunca sale de la zona → no emite; y el wiggle sigue desarmando.
static void test_safety_zone_wiggle_within_zone() {
  Machine m;
  Config c = defaults();
  m.reset(c, 0);
  uint32_t t0 = armDirect(m);
  std::vector<Step> s;
  uint32_t t = 2000000;
  int dir = +1;
  for (int rev = 0; rev < 5; rev++) {  // 5 tramos de 2 clicks alternando → 4 reversas
    for (int k = 0; k < 2; k++) { s.push_back({t, In::Wheel, dir}); t += 100000; }
    dir = -dir;
  }
  std::vector<Rec> r;
  run(m, s, r, t0);
  int stims = 0;
  for (auto &x : r) stims += x.a.act == Act::Stim;
  TEST_ASSERT_EQUAL_INT_MESSAGE(0, stims, "vaivén dentro de la zona no emite");
  TEST_ASSERT_FALSE_MESSAGE(m.armed(), "el wiggle dentro de la zona sigue desarmando");
}

// Girar sin parar durante el pulso (~20 clicks/s) NO dispara el nivel: sube PASO por pulso emitido, no por click.
static void test_ramp_level_per_pulse_not_per_click() {
  Machine m;
  Config c = defaults();
  m.reset(c, 0);
  std::vector<Step> s;
  uint32_t t0 = armDirect(m);
  for (int i = 0; i < 80; i++) s.push_back({2000000u + (uint32_t)i * 50000u, In::Wheel, +1});  // 4 s a 20 clicks/s
  std::vector<Rec> r;
  run(m, s, r, t0);
  std::vector<uint8_t> lv;
  for (auto &x : r)
    if (x.a.act == Act::Stim) lv.push_back(x.a.power);
  TEST_ASSERT_TRUE_MESSAGE(lv.size() >= 3, "varios pulsos a la cadencia");
  for (size_t k = 0; k < lv.size(); k++)
    TEST_ASSERT_EQUAL_UINT8_MESSAGE(c.minRamp + (k + 1) * c.step, lv[k], "cada pulso sube exactamente PASO");
  TEST_ASSERT_EQUAL_INT(-1, check(r, c).index);
}

static int tonoAfterMenu(bool resetOnExit) {
  Machine m;
  Config c = defaults();
  m.reset(c, 0);
  Action o[8];
  m.feed(In::KeyDown, 0, 100000, o, 8);  // combinación para abrir el menú
  m.feed(In::BtnDown, 0, 100000, o, 8);
  // ... menú abierto: los Up van a LVGL, no a la máquina ...
  if (resetOnExit) m.reset(c, 5000000);
  std::vector<Step> s;
  for (int i = 0; i < 5; i++) s.push_back({6000000u + i * 150000u, In::Wheel, (i & 1) ? -1 : 1});  // re-armar
  std::vector<Rec> r;
  run(m, s, r, 5000000);
  int tono = 0;
  for (auto &x : r) tono += (x.a.act == Act::Stim && x.a.mode == Mode::Tono);
  return tono;
}
static void test_menu_exit_reset_clears_held_keys() {
  TEST_ASSERT_EQUAL_INT_MESSAGE(0, tonoAfterMenu(true), "tras salir del menú (reset) no debe salir tono");
  TEST_ASSERT_TRUE_MESSAGE(tonoAfterMenu(false) > 0, "sin reset la KEY queda pegada (peligro real)");
}

// Interruptor de shock: apagado → todo shock sale como vibrar; encendido → shock. Los otros modos no cambian.
static void test_shock_switch_substitution() {
  TEST_ASSERT_EQUAL_INT((int)Mode::Vibrar, (int)effectiveMode(Mode::Shock, false));
  TEST_ASSERT_EQUAL_INT((int)Mode::Shock, (int)effectiveMode(Mode::Shock, true));
  for (Mode m : {Mode::Vibrar, Mode::Tono, Mode::Luz, Mode::None})
    for (bool on : {false, true}) TEST_ASSERT_EQUAL_INT((int)m, (int)effectiveMode(m, on));
}

int main() {
  UNITY_BEGIN();
  RUN_TEST(test_arm_via_wiggle);
  RUN_TEST(test_few_reversals_dont_arm);
  RUN_TEST(test_arm_grace_suppresses_immediate);
  RUN_TEST(test_click_while_disarmed_is_silent);
  RUN_TEST(test_single_click_low);
  RUN_TEST(test_double_click_high_no_low);
  RUN_TEST(test_ramp_discrete_pulses_spaced);
  RUN_TEST(test_ramp_fast_spin_drops_and_stops);
  RUN_TEST(test_ramp_caps_at_top);
  RUN_TEST(test_ramp_rate_capped_per_minute);
  RUN_TEST(test_rate_limit_suppresses_second);
  RUN_TEST(test_key_tono_repeats_and_caps);
  RUN_TEST(test_auto_disarm);
  RUN_TEST(test_auto_disarm_can_be_disabled);
  RUN_TEST(test_boots_disarmed);
  RUN_TEST(test_timelines_satisfy_invariants);
  RUN_TEST(test_fuzz_random_sequences);
  RUN_TEST(test_mutation_checker_has_teeth);
  RUN_TEST(test_menu_exit_reset_clears_held_keys);
  RUN_TEST(test_shock_switch_substitution);
  RUN_TEST(test_safety_zone_two_clicks_silent);
  RUN_TEST(test_safety_zone_third_click_starts_ramp);
  RUN_TEST(test_safety_zone_resets_after_pause);
  RUN_TEST(test_safety_zone_wiggle_within_zone);
  RUN_TEST(test_ramp_level_per_pulse_not_per_click);
  return UNITY_END();
}
