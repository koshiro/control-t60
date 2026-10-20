// Pruebas en el host: pio test -e native. Vectores generados desde collar_sub.py (tools/gen_vectors.py).
#include <math.h>
#include <stdio.h>
#include <string.h>
#include <unity.h>

#include "t60frame.h"
#include "vectors.h"
#if __has_include("vectors_local.h")
#include "vectors_local.h"
#endif

using namespace t60;

void setUp() {}
void tearDown() {}

static Frame toFrame(const FrameVec &v) {
  Frame f;
  f.seq = v.seq;
  f.mode = v.mode;
  f.power = v.power;
  f.idHi = v.idHi;
  f.idLo = v.idLo;
  f.chkXor = v.chkXor;
  return f;
}

// Cada trama, bit a bit contra frame_bits() de la referencia.
static void test_frame_bits_match_reference() {
  uint8_t bits[kFrameBits];
  char msg[160];
  for (const auto &v : kFrameVecs) {
    TEST_ASSERT_EQUAL_INT(kFrameBits, (int)strlen(v.bits));
    frameBits(toFrame(v), bits);
    for (int i = 0; i < kFrameBits; i++) {
      if (bits[i] != v.bits[i] - '0') {
        snprintf(msg, sizeof msg, "%s: bit %d difiere", v.label, i);
        TEST_FAIL_MESSAGE(msg);
      }
    }
  }
}

// Los 6 bytes del payload, reconstruidos desde los bits de la referencia.
static void test_payload_bytes_match_reference() {
  for (const auto &v : kFrameVecs) {
    uint8_t got[6], want[6];
    payload(toFrame(v), got);
    const char *p = v.bits + kPreambleBits + kSyncBits;
    for (int b = 0; b < 6; b++) {
      want[b] = 0;
      for (int k = 0; k < 8; k++) want[b] = (want[b] << 1) | (p[b * 8 + k] - '0');
    }
    TEST_ASSERT_EQUAL_HEX8_ARRAY_MESSAGE(want, got, 6, v.label);
  }
}

// Las pruebas negativas deben producir un checksum que el collar rechace (≠ Sum8 de su propio contenido
// para chkXor; ID distinto del collar para el ID ajeno).
static void test_negative_vectors_are_invalid() {
  int neg = 0;
  for (const auto &v : kFrameVecs) {
    if (strncmp(v.label, "NEG", 3)) continue;
    neg++;
    uint8_t p[6];
    payload(toFrame(v), p);
    bool chkOk = (uint8_t)(p[0] + p[1] + p[2] + p[3] + p[4]) == p[5];
    bool idOk = p[0] == kIdHi && p[1] == kIdLo;
    TEST_ASSERT_FALSE_MESSAGE(chkOk && idOk, v.label);
  }
  TEST_ASSERT_EQUAL_INT(2, neg);
}

// Pulsación completa contra los .sub que el collar YA obedeció (generados con gen(), enviados con Sor3nt).
// El .sub redondea cada corrida a µs enteros → tolerancia 0.5 µs por corrida.
// Los .sub llevan el ID real → viven en vectors_local.h (no versionado); sin él, la prueba se salta.
static void test_press_matches_collar_proven_sub() {
#if !__has_include("vectors_local.h")
  TEST_IGNORE_MESSAGE("sin vectors_local.h (capturas reales con el ID del collar; correr tools/gen_vectors.py)");
#else
  static Run runs[16000];
  char msg[200];
  for (const auto &g : kGolden) {
    Frame f;
    f.idHi = g.idHi;
    f.idLo = g.idLo;
    f.seq = g.seq;
    f.mode = g.mode;
    f.power = g.power;
    int n = pressRuns(f, g.frames, g.release, runs, 16000);
    snprintf(msg, sizeof msg, "%s: %d corridas, .sub tiene %d", g.file, n, g.n);
    TEST_ASSERT_EQUAL_INT_MESSAGE(g.n, n, msg);
    for (int i = 0; i < n; i++) {
      double us = runs[i].tenthsUs / 10.0;
      int32_t want = g.raw[i];
      bool levelOk = (want > 0) == (runs[i].level == 1);
      if (!levelOk || fabs(us - fabs((double)want)) > 0.5) {
        snprintf(msg, sizeof msg, "%s corrida %d: nivel %d %.1f µs, .sub %ld", g.file, i, runs[i].level, us,
                 (long)want);
        TEST_FAIL_MESSAGE(msg);
      }
    }
  }
#endif
}

static void test_press_overflow_reports_error() {
  Run runs[100];
  Frame f;
  TEST_ASSERT_EQUAL_INT(-1, pressRuns(f, 1, true, runs, 100));
}

// Duraciones que reporta el plan: trama 1104 bits = 115.04 ms, período 126.04 ms.
static void test_frame_timing_constants() {
  TEST_ASSERT_EQUAL_INT(1104, kFrameBits);
  TEST_ASSERT_EQUAL_UINT32(1150368, (uint32_t)kFrameBits * kBitTenthsUs);
  TEST_ASSERT_EQUAL_UINT32(1260368, (uint32_t)kFrameBits * kBitTenthsUs + kGapTenthsUs);
}

// rampRuns con un solo nivel y perFrame tramas == pressRuns de ese nivel (equivalencia estructural).
static void test_rampRuns_matches_pressRuns_single_level() {
  static Run a[16000], b[16000];
  Frame f;
  f.seq = 0x33;
  f.mode = kVibrar | 1;
  f.power = 42;
  int na = pressRuns(f, 3, true, a, 16000);
  uint8_t p = 42;
  int nb = rampRuns(0x33, kVibrar | 1, &p, 1, 3, true, b, 16000);
  TEST_ASSERT_EQUAL_INT(na, nb);
  for (int i = 0; i < na; i++) {
    TEST_ASSERT_EQUAL_UINT8(a[i].level, b[i].level);
    TEST_ASSERT_EQUAL_UINT32(a[i].tenthsUs, b[i].tenthsUs);
  }
}

// rampRuns multinivel: nº de tramas = niveles*perFrame + 1 (soltar), contadas por los silencios de 11 ms.
static void test_rampRuns_multi_level_frame_count() {
  static Run r[16000];
  uint8_t ps[3] = {10, 40, 80};
  int n = rampRuns(0x33, kVibrar | 1, ps, 3, 2, true, r, 16000);
  TEST_ASSERT_TRUE(n > 0);
  int gaps = 0;
  for (int i = 0; i < n; i++)
    if (r[i].level == 0 && r[i].tenthsUs >= kGapTenthsUs) gaps++;
  TEST_ASSERT_EQUAL_INT(3 * 2 + 1, gaps);
}

int main() {
  UNITY_BEGIN();
  RUN_TEST(test_rampRuns_matches_pressRuns_single_level);
  RUN_TEST(test_rampRuns_multi_level_frame_count);
  RUN_TEST(test_frame_bits_match_reference);
  RUN_TEST(test_payload_bytes_match_reference);
  RUN_TEST(test_negative_vectors_are_invalid);
  RUN_TEST(test_press_matches_collar_proven_sub);
  RUN_TEST(test_press_overflow_reports_error);
  RUN_TEST(test_frame_timing_constants);
  return UNITY_END();
}
