// Pruebas host del mapeo anillo → clicks (lib/ringmap): pio test -e native -f test_ringmap
#include <unity.h>

#include <random>
#include <vector>

#include "ringmap.h"

using namespace ringmap;

void setUp() {}
void tearDown() {}

// Reportes de ejemplo: tecla multimedia (un reporte) y gesto táctil guionado (varios reportes).
static uint32_t R(uint16_t src, std::initializer_list<uint8_t> b) {
  std::vector<uint8_t> v(b);
  return reportHash(src, v.data(), v.size());
}
static const uint32_t kPrev = R(0x2A, {0xB6, 0x00}), kNext = R(0x2A, {0xB5, 0x00}), kPlay = R(0x2A, {0xCD, 0x00});
static const std::vector<uint32_t> kSwipeUp = {R(0x2E, {1, 10, 80}), R(0x2E, {1, 10, 60}), R(0x2E, {1, 10, 40})};
static const std::vector<uint32_t> kSwipeDn = {R(0x2E, {1, 10, 40}), R(0x2E, {1, 10, 60}), R(0x2E, {1, 10, 80})};
static const std::vector<uint32_t> kTap = {R(0x2E, {1, 50, 50})};
static const std::vector<uint32_t> kLike = {R(0x2E, {1, 50, 50}), R(0x2E, {1, 50, 50})};  // doble toque propio

static Learn L(Table &t, const std::vector<uint32_t> &b, uint8_t btn, uint8_t clicks = 1) {
  return learn(t, b.data(), (int)b.size(), btn, clicks);
}
static std::vector<uint8_t> D(const Table &t, const std::vector<uint32_t> &b) {
  uint8_t out[8];
  int n = decode(t, b.data(), (int)b.size(), out, 8);
  return std::vector<uint8_t>(out, out + (n > 8 ? 8 : n));
}
static std::vector<uint32_t> cat(std::initializer_list<std::vector<uint32_t>> parts) {
  std::vector<uint32_t> v;
  for (auto &p : parts) v.insert(v.end(), p.begin(), p.end());
  return v;
}

static Table trained() {
  Table t;
  clear(t);
  // modo música: teclas · modo TikTok: gestos. Los dos modos apuntan a los mismos botones.
  TEST_ASSERT_EQUAL(Learn::Added, L(t, {kPrev}, 0));
  TEST_ASSERT_EQUAL(Learn::Added, L(t, {kPlay}, 1));
  TEST_ASSERT_EQUAL(Learn::Added, L(t, {kNext}, 2));
  TEST_ASSERT_EQUAL(Learn::Added, L(t, kSwipeDn, 0));
  TEST_ASSERT_EQUAL(Learn::Added, L(t, kTap, 1));
  TEST_ASSERT_EQUAL(Learn::Added, L(t, kSwipeUp, 2));
  return t;
}

static void test_release_reports_ignored() {
  uint8_t z[3] = {0, 0, 0}, nz[3] = {0, 1, 0};
  TEST_ASSERT_TRUE(isRelease(z, 3));
  TEST_ASSERT_TRUE(isRelease(z, 0));
  TEST_ASSERT_FALSE(isRelease(nz, 3));
  TEST_ASSERT_NOT_EQUAL(reportHash(1, nz, 3), reportHash(2, nz, 3));  // la característica cuenta
  TEST_ASSERT_NOT_EQUAL(reportHash(1, nz, 3), reportHash(1, nz, 2));
}

static void test_same_button_in_every_mode() {
  Table t = trained();
  TEST_ASSERT_TRUE(D(t, {kPrev}) == std::vector<uint8_t>{0});
  TEST_ASSERT_TRUE(D(t, kSwipeDn) == std::vector<uint8_t>{0});
  TEST_ASSERT_TRUE(D(t, {kNext}) == std::vector<uint8_t>{2});
  TEST_ASSERT_TRUE(D(t, kSwipeUp) == std::vector<uint8_t>{2});
  TEST_ASSERT_TRUE(D(t, kTap) == std::vector<uint8_t>{1});
}

static void test_fast_clicks_merge_into_one_burst() {
  Table t = trained();
  TEST_ASSERT_TRUE(D(t, {kPrev, kPrev}) == (std::vector<uint8_t>{0, 0}));
  TEST_ASSERT_TRUE(D(t, cat({kSwipeUp, kSwipeUp, kSwipeUp})) == (std::vector<uint8_t>{2, 2, 2}));
  TEST_ASSERT_TRUE(D(t, kLike) == (std::vector<uint8_t>{1, 1}));  // doble toque = 2 × toque
}

static void test_unknown_burst_is_silent() {
  Table t = trained();
  uint32_t vol = R(0x2A, {0xE9, 0x00});  // pulsación larga: volumen, nunca aprendido
  TEST_ASSERT_EQUAL(0, (int)D(t, {vol}).size());
  TEST_ASSERT_EQUAL(0, (int)D(t, {kPrev, vol}).size());  // un trozo desconocido anula toda la ráfaga
  TEST_ASSERT_EQUAL(0, (int)D(t, {vol, kPrev}).size());
  TEST_ASSERT_EQUAL(0, (int)D(t, {kSwipeUp[0], kSwipeUp[1]}).size());  // gesto cortado
  Table e;
  clear(e);
  TEST_ASSERT_EQUAL(0, (int)D(e, {kPrev}).size());
}

static void test_learn_known_conflict_double() {
  Table t = trained();
  uint8_t before = t.count;
  TEST_ASSERT_EQUAL(Learn::Known, L(t, {kPrev}, 0));             // repetido (mismo reporte en otro modo)
  TEST_ASSERT_EQUAL(Learn::Known, L(t, {kPrev, kPrev}, 0, 2));   // doble = 2 × simple
  TEST_ASSERT_EQUAL(Learn::Conflict, L(t, {kPrev}, 2));          // el mismo reporte no puede ser otro botón
  TEST_ASSERT_EQUAL(Learn::Conflict, L(t, {kPrev}, 0, 2));       // ni otra cantidad de clicks
  TEST_ASSERT_EQUAL(Learn::Empty, L(t, {}, 0));
  TEST_ASSERT_EQUAL(before, t.count);
  uint32_t like = R(0x2A, {0x40, 0x00});                          // reporte PROPIO para el doble toque
  TEST_ASSERT_EQUAL(Learn::Added, L(t, {like}, 1, 2));
  TEST_ASSERT_TRUE(D(t, {like}) == (std::vector<uint8_t>{1, 1}));
}

static void test_settled_only_when_unambiguous() {
  Table t = trained();
  auto S = [&](const std::vector<uint32_t> &b) { return settled(t, b.data(), (int)b.size()); };
  TEST_ASSERT_TRUE(S({kPrev}));                    // un reporte, nada más largo empieza igual → decidir ya
  TEST_ASSERT_TRUE(S({kPrev, kPrev}));
  TEST_ASSERT_FALSE(S({kSwipeUp[0]}));             // gesto a medias: esperar
  TEST_ASSERT_FALSE(S({kSwipeUp[0], kSwipeUp[1]}));
  TEST_ASSERT_TRUE(S(kSwipeUp));                   // gesto completo
  TEST_ASSERT_FALSE(S({R(0x2A, {0xE9, 0x00})}));   // desconocido: nunca "listo"
  // una firma larga que EMPIEZA igual que una corta: la corta ya no se decide sola
  uint32_t a = R(0x30, {1}), b = R(0x30, {2});
  TEST_ASSERT_EQUAL(Learn::Added, L(t, {a}, 0));
  TEST_ASSERT_TRUE(S({a}));
  TEST_ASSERT_EQUAL(Learn::Added, L(t, {a, b, b}, 2));
  TEST_ASSERT_FALSE(S({a}));
  TEST_ASSERT_TRUE(S({a, b, b}));
  TEST_ASSERT_TRUE(D(t, {a, b, b}) == std::vector<uint8_t>{2});
}

static void test_table_full_and_validation() {
  Table t;
  clear(t);
  for (int i = 0; i < kMaxSigs; i++) TEST_ASSERT_EQUAL(Learn::Added, L(t, {R(1, {(uint8_t)(i + 1)})}, i % 3));
  TEST_ASSERT_EQUAL(Learn::Full, L(t, {R(1, {200})}, 0));
  TEST_ASSERT_TRUE(valid(t));
  Table bad = t;
  bad.sigs[3].button = 7;
  TEST_ASSERT_FALSE(valid(bad));
  TEST_ASSERT_EQUAL(0, (int)D(bad, {R(1, {1})}).size());  // tabla corrupta → nada
  bad = t;
  bad.version = 99;
  TEST_ASSERT_FALSE(valid(bad));
  bad = t;
  bad.count = kMaxSigs + 1;
  TEST_ASSERT_FALSE(valid(bad));
}

static void test_burst_assembly() {
  Burst b;
  TEST_ASSERT_FALSE(b.closed(1000000));
  b.add(kPrev, 1000);
  b.add(kPrev, 1000 + kBurstGapUs - 1);  // dentro del silencio → misma ráfaga
  TEST_ASSERT_FALSE(b.closed(1000 + 2 * kBurstGapUs - 2));
  TEST_ASSERT_TRUE(b.closed(1000 + 2 * kBurstGapUs));
  TEST_ASSERT_EQUAL(2, b.size());
  b.reset();
  TEST_ASSERT_TRUE(b.empty());
  for (int i = 0; i < kMaxBurst + 5; i++) b.add(kPrev, 5000 + i);
  TEST_ASSERT_TRUE(b.overflow());
  TEST_ASSERT_EQUAL(0, b.size());  // desbordada → no decodifica nada
  // wrap de micros()
  Burst w;
  w.add(kPrev, 0xFFFFFF00u);
  TEST_ASSERT_FALSE(w.closed(0xFFFFFF00u + 1000));
  TEST_ASSERT_TRUE(w.closed((uint32_t)(0xFFFFFF00u + kBurstGapUs)));
}

// Ráfagas al azar contra una tabla entrenada: decode solo devuelve botones válidos y nunca más clicks que
// reportes × 2; y toda ráfaga hecha SOLO de reportes nunca aprendidos da 0.
static void test_fuzz_decode() {
  Table t = trained();
  std::mt19937 rng(0x51A6);
  std::vector<uint32_t> pool = {kPrev, kNext, kPlay, kTap[0], kSwipeUp[0], kSwipeUp[1], kSwipeUp[2]};
  for (int it = 0; it < 5000; it++) {
    std::vector<uint32_t> b;
    int n = 1 + rng() % 12;
    bool onlyUnknown = rng() % 4 == 0;
    for (int i = 0; i < n; i++) b.push_back(onlyUnknown ? (uint32_t)rng() | 1u : pool[rng() % pool.size()]);
    uint8_t out[32];
    int c = decode(t, b.data(), n, out, 32);
    TEST_ASSERT_TRUE(c >= 0 && c <= 2 * n);
    for (int i = 0; i < c && i < 32; i++) TEST_ASSERT_TRUE(out[i] < kButtons);
    if (onlyUnknown) TEST_ASSERT_EQUAL(0, c);
  }
}

int main(int, char **) {
  UNITY_BEGIN();
  RUN_TEST(test_release_reports_ignored);
  RUN_TEST(test_same_button_in_every_mode);
  RUN_TEST(test_fast_clicks_merge_into_one_burst);
  RUN_TEST(test_unknown_burst_is_silent);
  RUN_TEST(test_learn_known_conflict_double);
  RUN_TEST(test_settled_only_when_unambiguous);
  RUN_TEST(test_table_full_and_validation);
  RUN_TEST(test_burst_assembly);
  RUN_TEST(test_fuzz_decode);
  return UNITY_END();
}
