// Pruebas en el host de los ajustes editables (menú + NVS): pio test -e native.
// Lo que importa para la seguridad: nada que venga del menú o de la NVS (vacía, de otro esquema, corrupta) puede
// dejar la cadencia bajo el piso medido ni un nivel sobre el TOPE.
#include <unity.h>

#include <map>
#include <random>
#include <string>

#include "settings.h"

using namespace settings;

void setUp() {}
void tearDown() {}

struct FakeStore : Store {
  std::map<std::string, int32_t> kv;
  bool get(const char *k, int32_t &v) override {
    auto it = kv.find(k);
    if (it == kv.end()) return false;
    v = it->second;
    return true;
  }
  bool put(const char *k, int32_t v) override {
    kv[k] = v;
    return true;
  }
};

static void assertSafe(const gesture::Config &c) {
  for (int i = 0; i < kCount; i++) {
    int32_t v = get(c, i);
    TEST_ASSERT_TRUE_MESSAGE(v >= field(i).min && v <= field(i).max, field(i).key);
    TEST_ASSERT_EQUAL_INT_MESSAGE(0, (v - field(i).min) % field(i).step, field(i).key);
  }
  TEST_ASSERT_TRUE(c.minStimGapUs >= 500000);
  TEST_ASSERT_TRUE(c.lowPower <= c.cap && c.highPower <= c.cap && c.minRamp <= c.cap);
  TEST_ASSERT_TRUE(c.keyVibrPower <= c.cap && c.keyTonoPower <= c.cap);
}

// Abrir el menú y guardar sin tocar nada no debe cambiar la config.
static void test_defaults_are_fixed_point() {
  gesture::Config d = gesture::defaults(), n = d;
  normalize(n);
  for (int i = 0; i < kCount; i++) TEST_ASSERT_EQUAL_INT_MESSAGE(get(d, i), get(n, i), field(i).key);
  TEST_ASSERT_EQUAL_UINT8(d.keyTonoPower, n.keyTonoPower);
}

static void test_clamp_and_step() {
  TEST_ASSERT_EQUAL_INT(500, clampField(0, 0));
  TEST_ASSERT_EQUAL_INT(500, clampField(0, -5));
  TEST_ASSERT_EQUAL_INT(2000, clampField(0, 99999));
  TEST_ASSERT_EQUAL_INT(960, clampField(0, 951));
  TEST_ASSERT_EQUAL_INT(940, clampField(0, 949));
  TEST_ASSERT_EQUAL_INT(1, clampField(2, 0));     // TOPE nunca 0
  TEST_ASSERT_EQUAL_INT(2, clampField(6, 1));     // wiggle de 1 reversa armaría con cualquier vaivén
  TEST_ASSERT_EQUAL_INT(3000, clampField(7, 2999));
}

// Bajar el TOPE arrastra los demás niveles (lo que se ve en el menú = lo que usa la máquina).
static void test_normalize_enforces_cap() {
  gesture::Config c = gesture::defaults();
  c.cap = 10;
  c.lowPower = 50;
  c.highPower = 99;
  c.minRamp = 20;
  c.keyVibrPower = 30;
  c.keyTonoPower = 40;
  normalize(c);
  TEST_ASSERT_EQUAL_UINT8(10, c.lowPower);
  TEST_ASSERT_EQUAL_UINT8(10, c.highPower);
  TEST_ASSERT_EQUAL_UINT8(10, c.minRamp);
  TEST_ASSERT_EQUAL_UINT8(10, c.keyVibrPower);
  TEST_ASSERT_EQUAL_UINT8(10, c.keyTonoPower);
  // y coincide con lo que haría la máquina
  gesture::Machine m;
  m.reset(c, 0);
  for (int i = 0; i < kCount; i++) TEST_ASSERT_EQUAL_INT(get(c, i), get(m.config(), i));
}

static void test_load_empty_is_defaults() {
  FakeStore s;
  int found = -1;
  gesture::Config c = load(s, gesture::defaults(), &found);
  TEST_ASSERT_EQUAL_INT(0, found);
  for (int i = 0; i < kCount; i++) TEST_ASSERT_EQUAL_INT(get(gesture::defaults(), i), get(c, i));
}

static void test_save_load_roundtrip() {
  gesture::Config c = gesture::defaults();
  set(c, 0, 1200);
  set(c, 1, 3);
  set(c, 2, 40);
  set(c, 3, 22);
  set(c, 4, 7);
  set(c, 5, 33);
  set(c, 6, 5);
  set(c, 7, 1500);
  FakeStore s;
  TEST_ASSERT_TRUE(save(s, c));
  int found = 0;
  gesture::Config r = load(s, gesture::defaults(), &found);
  TEST_ASSERT_EQUAL_INT(kCount, found);
  for (int i = 0; i < kCount; i++) TEST_ASSERT_EQUAL_INT_MESSAGE(get(c, i), get(r, i), field(i).key);
}

static void test_wrong_schema_ignored() {
  FakeStore s;
  s.kv["schema"] = kSchema + 1;
  s.kv["gap"] = 1800;
  s.kv["cap"] = 90;
  int found = -1;
  gesture::Config c = load(s, gesture::defaults(), &found);
  TEST_ASSERT_EQUAL_INT(0, found);
  TEST_ASSERT_EQUAL_UINT32(gesture::defaults().minStimGapUs, c.minStimGapUs);
  TEST_ASSERT_EQUAL_UINT8(gesture::defaults().cap, c.cap);
}

// NVS corrupta: cadencia 0, TOPE 255, niveles fuera de rango → todo recortado.
static void test_corrupt_store_clamped() {
  FakeStore s;
  s.kv = {{"schema", kSchema}, {"gap", 0},  {"minr", -3}, {"cap", 255},  {"vib", 1000},
          {"lo", -1},          {"hi", 500}, {"wig", 0},   {"grace", -9}, {"zona", 99}};
  gesture::Config c = load(s, gesture::defaults());
  assertSafe(c);
  TEST_ASSERT_EQUAL_UINT32(500000, c.minStimGapUs);
  TEST_ASSERT_EQUAL_UINT8(99, c.cap);
  TEST_ASSERT_EQUAL_UINT8(2, c.wiggleRevs);
  TEST_ASSERT_EQUAL_UINT8(4, c.safeClicks);
}

// Tiempo de bloqueo (auto-desarme) en segundos, 10-600; el interruptor de auto-desarme persiste on/off.
static void test_auto_disarm_fields() {
  gesture::Config c = gesture::defaults();
  TEST_ASSERT_EQUAL_INT(1, get(c, 10));
  TEST_ASSERT_EQUAL_INT(30, get(c, 11));
  set(c, 10, 0);
  set(c, 11, 125);  // → 130 (paso 10)
  TEST_ASSERT_FALSE(c.autoDisarm);
  TEST_ASSERT_EQUAL_UINT32(130000000u, c.autoDisarmUs);
  set(c, 11, 5);
  TEST_ASSERT_EQUAL_UINT32(10000000u, c.autoDisarmUs);
  set(c, 11, 100000);
  TEST_ASSERT_EQUAL_UINT32(600000000u, c.autoDisarmUs);
  FakeStore s;
  save(s, c);
  gesture::Config r = load(s, gesture::defaults());
  TEST_ASSERT_FALSE(r.autoDisarm);
  TEST_ASSERT_EQUAL_UINT32(600000000u, r.autoDisarmUs);
  s.kv["adis"] = 7;  // basura → cualquier distinto de 0 = activado (lo seguro)
  TEST_ASSERT_TRUE(load(s, gesture::defaults()).autoDisarm);
}

// Fuzz: valores arbitrarios (incluidos extremos de int32) en NVS y en el menú → siempre seguro.
static void test_fuzz_store_and_menu() {
  std::mt19937 rng(1234);
  std::uniform_int_distribution<int32_t> any(INT32_MIN, INT32_MAX), small(-50, 3500);
  for (int it = 0; it < 3000; it++) {
    FakeStore s;
    s.kv["schema"] = kSchema;
    for (int i = 0; i < kCount; i++)
      if (rng() & 1) s.kv[field(i).key] = (rng() & 1) ? any(rng) : small(rng);
    gesture::Config c = load(s, gesture::defaults());
    assertSafe(c);
    for (int k = 0; k < 6; k++) set(c, rng() % kCount, (rng() & 1) ? any(rng) : small(rng));  // edición en el menú
    normalize(c);
    assertSafe(c);
    FakeStore s2;
    save(s2, c);
    gesture::Config r = load(s2, gesture::defaults());
    for (int i = 0; i < kCount; i++) TEST_ASSERT_EQUAL_INT(get(c, i), get(r, i));
  }
}

static void test_light_mode_store() {
  FakeStore s;
  TEST_ASSERT_EQUAL_INT(1, loadLightMode(s));  // vacío → modo 1 (el verificado)
  save(s, gesture::defaults());                // escribe el esquema
  TEST_ASSERT_TRUE(saveLightMode(s, 3));
  TEST_ASSERT_EQUAL_INT(3, loadLightMode(s));
  s.kv["lmode"] = 0;  // corrupto: 0 sería "apagar", no un modo
  TEST_ASSERT_EQUAL_INT(1, loadLightMode(s));
  s.kv["lmode"] = 99;
  TEST_ASSERT_EQUAL_INT(3, loadLightMode(s));
  s.kv["schema"] = kSchema + 1;
  s.kv["lmode"] = 2;
  TEST_ASSERT_EQUAL_INT(1, loadLightMode(s));
}

// El interruptor de shock persiste; sin dato o con basura → apagado (falla hacia lo seguro).
static void test_shock_switch_persists() {
  FakeStore s;
  TEST_ASSERT_FALSE(loadShock(s));
  TEST_ASSERT_TRUE(saveShock(s, true));
  TEST_ASSERT_TRUE(loadShock(s));  // sin esquema guardado igual persiste
  TEST_ASSERT_TRUE(saveShock(s, false));
  TEST_ASSERT_FALSE(loadShock(s));
  for (int32_t bad : {2, -1, 255, INT32_MIN}) {
    s.kv["shock"] = bad;
    TEST_ASSERT_FALSE(loadShock(s));
  }
  // guardar el menú no toca el interruptor
  saveShock(s, true);
  save(s, gesture::defaults());
  TEST_ASSERT_TRUE(loadShock(s));
}

int main() {
  UNITY_BEGIN();
  RUN_TEST(test_defaults_are_fixed_point);
  RUN_TEST(test_clamp_and_step);
  RUN_TEST(test_normalize_enforces_cap);
  RUN_TEST(test_load_empty_is_defaults);
  RUN_TEST(test_save_load_roundtrip);
  RUN_TEST(test_wrong_schema_ignored);
  RUN_TEST(test_corrupt_store_clamped);
  RUN_TEST(test_fuzz_store_and_menu);
  RUN_TEST(test_light_mode_store);
  RUN_TEST(test_auto_disarm_fields);
  RUN_TEST(test_shock_switch_persists);
  return UNITY_END();
}
