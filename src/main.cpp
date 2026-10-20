// Control T60 — fase 1: TX 2-FSK del collar T60 desde comandos seriales, con verificación en el equipo.
// Equipo propio de Daniel (collar Abbidot T60 de su perro). Plan: references/control-firmware.md del skill.
//
// Comandos (una línea por USB CDC, 115200):
//   ping                                     -> "PONG control-t60 <PARTNUM> <VERSION>"
//   regs                                     -> vuelca 0x00–0x2E + PATABLE[0], compara con lo esperado
//   tx <luz|tono|vibrar|shock> <0-99> [tramas 1-6] [badchk|badid]
//                                            -> una pulsación (tramas + soltar) y mide el GPIO3 mientras sale
//   wdtest                                   -> prueba el watchdog: TX que debe cortarse a los 40 ms
//
// Seguridad: shock tras DOS barreras: compilación (ALLOW_SHOCK=1 lo permite) + interruptor `shock on|off` por serial
// (NO en la GUI), persistente en NVS; apagado → shock sale como vibrar. El CC1101 arranca y vuelve siempre a IDLE;
// watchdog (tarea FreeRTOS en el core 0) fuerza SIDLE si una pulsación pasa de lo esperado; "soltar" al final de toda TX.

#include <Arduino.h>
#include <SPI.h>
#include <driver/rmt_tx.h>
#include <esp_random.h>
#include <soc/gpio_reg.h>
#include <soc/io_mux_reg.h>
#include <xtensa/core-macros.h>

#include <algorithm>
#include <esp_heap_caps.h>
#include <math.h>
#include <vector>

#include <driver/i2s_std.h>
#include <driver/pulse_cnt.h>

#include "gesture.h"
#include "lcd.h"
#include "settings.h"

#include <Preferences.h>
#include <lvgl.h>
#include "t60frame.h"

#ifndef ALLOW_SHOCK
#define ALLOW_SHOCK 0
#endif
// Interruptor de shock en tiempo de ejecución. Solo se cambia por serial (`shock on|off`), nunca desde la GUI; se
// guarda en NVS al cambiarlo y se carga al arrancar. Con ALLOW_SHOCK=0 nunca queda encendido.
static bool g_shockOn = false;
static inline bool shockEnabled() { return ALLOW_SHOCK && g_shockOn; }

// ---- pines: utilities.h del repo oficial Xinyuan-LilyGO/T-Embed-CC1101 (coincide con el header de Sor3nt) ----
#define PIN_SCK    11
#define PIN_MISO   10
#define PIN_MOSI    9
#define PIN_CSN    12   // BOARD_LORA_CS
#define PIN_GDO0    3   // BOARD_LORA_IO0: dato TX en modo asíncrono (lo maneja el RMT)
#define PIN_GDO2   38   // BOARD_LORA_IO2: sin uso, entrada
#define PIN_PWR_EN 15   // BOARD_PWR_EN: alimenta el módulo RF
#define PIN_SW1    47   // BOARD_LORA_SW1 → RF_SW_1
#define PIN_SW0    48   // BOARD_LORA_SW0 → RF_SW_0. Esquemático XY24A2 v2.1: SW1=1,SW0=1 → filtro 433.92 MHz
#define PIN_LCD_CS 41
#define PIN_SD_CS  13
#define PIN_NRF_CS 44
#define PIN_ENC_A   4   // encoder A (activo en bajo) — para el punto 2 (medición de gestos)
#define PIN_ENC_B   5   // encoder B
#define PIN_ENC_BTN 0   // botón del encoder = BOOT (strapping; se lee tras el arranque)
#define PIN_KEY     6   // botón lateral KEY

static constexpr double XTAL_HZ = 26e6;
static constexpr int MAX_FRAMES = 6;

// ---- CC1101: registros (SWRS061I §29) ----
enum : uint8_t {
  IOCFG2 = 0x00, IOCFG0 = 0x02, PKTCTRL0 = 0x08, FREQ2 = 0x0D, FREQ1 = 0x0E, FREQ0 = 0x0F,
  MDMCFG4 = 0x10, MDMCFG3 = 0x11, MDMCFG2 = 0x12, DEVIATN = 0x15, MCSM0 = 0x18, FREND0 = 0x22,
  FSCAL3 = 0x23, FSCAL2 = 0x24, FSCAL1 = 0x25, PATABLE = 0x3E,
  SRES = 0x30, STX = 0x35, SIDLE = 0x36,
  PARTNUM = 0x30, VERSION = 0x31, MARCSTATE = 0x35,
};
static constexpr uint8_t MARC_IDLE = 0x01, MARC_TX = 0x13;

static const char *const REG_NAMES[0x2F] = {
  "IOCFG2", "IOCFG1", "IOCFG0", "FIFOTHR", "SYNC1", "SYNC0", "PKTLEN", "PKTCTRL1", "PKTCTRL0", "ADDR",
  "CHANNR", "FSCTRL1", "FSCTRL0", "FREQ2", "FREQ1", "FREQ0", "MDMCFG4", "MDMCFG3", "MDMCFG2", "MDMCFG1",
  "MDMCFG0", "DEVIATN", "MCSM2", "MCSM1", "MCSM0", "FOCCFG", "BSCFG", "AGCCTRL2", "AGCCTRL1", "AGCCTRL0",
  "WOREVT1", "WOREVT0", "WORCTRL", "FREND1", "FREND0", "FSCAL3", "FSCAL2", "FSCAL1", "FSCAL0", "RCCTRL1",
  "RCCTRL0", "FSTEST", "PTEST", "AGCTEST", "TEST2", "TEST1", "TEST0",
};

// Valores de reset, columna "Reset" de §29.2 (SWRS061I págs. 71–92), byte armado desde cada campo.
static const uint8_t RESET_VALUES[0x2F] = {
  0x29, 0x2E, 0x3F, 0x07, 0xD3, 0x91, 0xFF, 0x04, 0x45, 0x00, 0x00, 0x0F, 0x00, 0x1E, 0xC4, 0xEC,
  0x8C, 0x22, 0x02, 0x22, 0xF8, 0x47, 0x07, 0x30, 0x04, 0x36, 0x6C, 0x03, 0x40, 0x91, 0x87, 0x6B,
  0xF8, 0x56, 0x10, 0xA9, 0x0A, 0x20, 0x0D, 0x41, 0x00, 0x59, 0x7F, 0x3F, 0x88, 0x31, 0x0B,
};

// Solo se escribe lo que TX 2-FSK asíncrono necesita; todo lo demás queda en reset (lo que leemos de vuelta
// debe ser exactamente reset ⊕ esto). Cada valor con su cálculo desde el datasheet:
static uint8_t g_cfg[][2] = {
  // GDO0 en alta impedancia. En reset GDO0 saca CLK_XOSC/192 (§26) y con 0x0D (preset Flipper) maneja el pin
  // fuera de TX → choque con el ESP. En modo serial GDO0 es entrada de datos en TX sin importar GDO0_CFG (§10).
  {IOCFG0, 0x2E},
  {IOCFG2, 0x2E},  // GDO2 sin uso → 3-state (reset 0x29 = CHP_RDYn manejando el pin 38)
  // WHITE_DATA=0, PKT_FORMAT=3 (asíncrono, dato por GDO0), CRC_EN=0, LENGTH_CONFIG=2 (infinito).
  {PKTCTRL0, 0x32},
  // FREQ = round(433.92e6 / 26e6 · 2^16) = 1093745 = 0x10B071 → 433 919 830 Hz (−170 Hz).
  {FREQ2, 0x10}, {FREQ1, 0xB0}, {FREQ0, 0x71},
  // §27.1: el modulador muestrea GDO0 a 8× la tasa programada y exige error de bit < 1/8.
  // R = (256+M)·2^E / 2^28 · fxosc con E=8, M=0x83 → 9595.9 Bd ≈ 1/104.2 µs; muestreo cada 13 µs.
  // (El preset Flipper 0x67 da E=7 → 4798 Bd: muestrea cada 26 µs, ¼ de bit.) CHANBW (bits 7:4) = reset.
  {MDMCFG4, 0x88}, {MDMCFG3, 0x83},
  // MDMCFG2 queda en reset 0x02: MOD_FORMAT=000 = 2-FSK.
  // f_dev = fxosc / 2^17 · (8+M) · 2^E con E=3, M=0 → 12 695 Hz (la captura: Dev12K).
  {DEVIATN, 0x30},
  // FS_AUTOCAL=01: calibra en IDLE→TX (FREQ cambió desde el reset). PO_TIMEOUT=10 (valor de SmartRF).
  {MCSM0, 0x18},
  // FREND0 queda en reset 0x10: PA_POWER=0 → usa PATABLE[0] (FSK no hace shaping).
};
// Tabla 39 (inductores multicapa), columna 433 MHz: 0xC0 = +10 dBm.
static const uint8_t PA_433_10DBM = 0xC0;

// El `SPI` global (FSPI): LovyanGFX re-inicia ese mismo objeto al iniciar el LCD (ver lcd.h) → CC1101 y LCD comparten
// el periférico y su mutex HAL. Un SPIClass(FSPI) aparte quedaría con estado viejo tras el reset del periférico.
static SPIClass &spi = SPI;
static const SPISettings SPI_CFG(4000000, MSBFIRST, SPI_MODE0);
static SemaphoreHandle_t g_spiLock;
// Serializa transmisiones completas (comandos de bench + tarea de gestos) para que no se solapen en el CC1101.
static SemaphoreHandle_t g_rfMutex;
static inline void rfLock() { xSemaphoreTake(g_rfMutex, portMAX_DELAY); }
static inline void rfUnlock() { xSemaphoreGive(g_rfMutex); }

// ---------------------------------------------------------------- driver CC1101 (base: capturador/src/main.cpp)
static void csLow() {
  digitalWrite(PIN_CSN, LOW);
  uint32_t t = micros();
  while (digitalRead(PIN_MISO) && micros() - t < 1000) {}  // CHIP_RDYn
}
static void csHigh() { digitalWrite(PIN_CSN, HIGH); }

static uint8_t xfer(uint8_t a, int n, const uint8_t *tx, uint8_t *rx) {
  xSemaphoreTake(g_spiLock, portMAX_DELAY);
  spi.beginTransaction(SPI_CFG);
  csLow();
  uint8_t st = spi.transfer(a);
  for (int i = 0; i < n; i++) {
    uint8_t v = spi.transfer(tx ? tx[i] : 0);
    if (rx) rx[i] = v;
  }
  csHigh();
  spi.endTransaction();
  xSemaphoreGive(g_spiLock);
  return st;
}
static uint8_t strobe(uint8_t s) { return xfer(s, 0, nullptr, nullptr); }
static void writeReg(uint8_t a, uint8_t v) { xfer(a, 1, &v, nullptr); }
static uint8_t readReg(uint8_t a) { uint8_t v; xfer(a | 0x80, 1, nullptr, &v); return v; }
static uint8_t readStatus(uint8_t a) { uint8_t v; xfer(a | 0xC0, 1, nullptr, &v); return v; }
static uint8_t marc() { return readStatus(MARCSTATE) & 0x1F; }
static uint8_t readPa0() { uint8_t v; xfer(PATABLE | 0xC0, 1, nullptr, &v); return v; }  // burst → PATABLE[0]

static bool waitMarc(uint8_t want, uint32_t ms) {
  uint32_t t = millis();
  while (marc() != want)
    if (millis() - t > ms) return false;
  return true;
}

static void cc1101Reset() {
  csHigh(); delayMicroseconds(5);
  digitalWrite(PIN_CSN, LOW); delayMicroseconds(10);
  csHigh(); delayMicroseconds(45);
  strobe(SRES);
  delay(1);
}

static void cc1101Config() {
  for (auto &r : g_cfg) writeReg(r[0], r[1]);
  writeReg(PATABLE, PA_433_10DBM);
  strobe(SIDLE);
  waitMarc(MARC_IDLE, 50);
}

// ---------------------------------------------------------------- RMT (dato TX en GDO0)
static rmt_channel_handle_t g_rmt;
static rmt_encoder_handle_t g_copy;
static constexpr uint32_t RMT_HZ = 10'000'000;  // 0.1 µs/tick → 104.2 µs = 1042 ticks exactos
static bool g_rmtDma;

static void rmtInit() {
  rmt_tx_channel_config_t c = {};
  c.gpio_num = (gpio_num_t)PIN_GDO0;
  c.clk_src = RMT_CLK_SRC_DEFAULT;
  c.resolution_hz = RMT_HZ;
  c.trans_queue_depth = 1;
  c.mem_block_symbols = 1024;
  c.flags.with_dma = 1;  // sin recargas por interrupción durante la trama
  g_rmtDma = rmt_new_tx_channel(&c, &g_rmt) == ESP_OK;
  if (!g_rmtDma) {
    c.flags.with_dma = 0;
    c.mem_block_symbols = 96;
    ESP_ERROR_CHECK(rmt_new_tx_channel(&c, &g_rmt));
  }
  rmt_copy_encoder_config_t ec = {};
  ESP_ERROR_CHECK(rmt_new_copy_encoder(&ec, &g_copy));
  ESP_ERROR_CHECK(rmt_enable(g_rmt));
  REG_SET_BIT(IO_MUX_GPIO3_REG, FUN_IE);  // entrada habilitada en el pad → GPIO_IN_REG ve lo que sale
}

static inline uint32_t gdo0Level() { return (REG_READ(GPIO_IN_REG) >> PIN_GDO0) & 1; }

// Corridas → símbolos RMT (cada símbolo lleva dos corridas; duración máx. 32767 ticks por mitad).
// rmt_transmit() arranca el hardware dentro de la llamada si el canal está libre (rmt_tx.h), así que el
// primer flanco se perdería antes de capture(). Se anteponen 2 ms en bajo: en RF es lo mismo que ya pasa
// tras STX (GDO0 bajo = f−dev), y el primer flanco cae dentro de la captura.
static constexpr uint16_t LEAD_TICKS = 20000;

static void toSymbols(const t60::Run *runs, int n, std::vector<rmt_symbol_word_t> &out) {
  std::vector<std::pair<uint8_t, uint16_t>> halves{{0, LEAD_TICKS}};
  for (int i = 0; i < n; i++) {
    uint32_t t = runs[i].tenthsUs;  // décimas de µs = ticks a 10 MHz
    while (t > 0) {
      uint16_t d = t > 30000 ? 30000 : t;
      if (t - d > 0 && t - d < 2) d -= 2;  // nunca dejar un trozo de 0/1 tick
      halves.push_back({runs[i].level, d});
      t -= d;
    }
  }
  if (halves.size() & 1) {  // par: partir el último silencio (siempre largo) en dos
    auto last = halves.back();
    halves.back().second = last.second / 2;
    halves.push_back({last.first, (uint16_t)(last.second - last.second / 2)});
  }
  out.clear();
  for (size_t i = 0; i < halves.size(); i += 2) {
    rmt_symbol_word_t s;
    s.level0 = halves[i].first;
    s.duration0 = halves[i].second;
    s.level1 = halves[i + 1].first;
    s.duration1 = halves[i + 1].second;
    out.push_back(s);
  }
}

// ---------------------------------------------------------------- watchdog
// Tarea propia en el core 0 (el loop corre en el core 1). Armar = notificar con el plazo en ms; desarmar =
// notificar con 0. Si vence el plazo sin desarme: SIDLE (apaga el PA, da igual qué haga GDO0). No toca el
// RMT; el loop lo limpia.
static TaskHandle_t g_wdTask;
static volatile bool g_wdFired;
static volatile uint32_t g_wdRuns;

static void wdTask(void *) {
  uint32_t ms;
  for (;;) {
    xTaskNotifyWait(0, UINT32_MAX, &ms, portMAX_DELAY);
    while (ms) {  // armado: esperar desarme (0) o vencimiento; un re-armado reinicia el plazo
      uint32_t next;
      if (xTaskNotifyWait(0, UINT32_MAX, &next, pdMS_TO_TICKS(ms)) == pdTRUE) {
        ms = next;
        continue;
      }
      g_wdRuns = g_wdRuns + 1;
      strobe(SIDLE);
      g_wdFired = true;
      ms = 0;
    }
  }
}
static void wdArm(uint32_t ms) { xTaskNotify(g_wdTask, ms, eSetValueWithOverwrite); }
static void wdDisarm() { xTaskNotify(g_wdTask, 0, eSetValueWithOverwrite); }

// ---------------------------------------------------------------- medición en el pad
struct Edge {
  uint32_t before, at;  // última muestra sin cambio y primera con cambio (ciclos de CPU)
};
static Edge *g_edges;
static constexpr int MAX_EDGES = 8200;  // 7 tramas × ~1080 cambios + margen
static uint32_t g_maxSampleGap;

// Muestrea GPIO3 en ciclo cerrado hasta `limitUs` desde la llamada. Devuelve cantidad de flancos.
static uint32_t g_startLevel;

static int capture(uint32_t limitUs, uint32_t &t0) {
  const uint32_t cpm = getCpuFrequencyMhz();
  uint32_t last = gdo0Level(), prev = XTHAL_GET_CCOUNT(), n = 0;
  g_startLevel = last;
  t0 = prev;
  g_maxSampleGap = 0;
  const uint32_t limit = limitUs * cpm;
  for (;;) {
    uint32_t now = XTHAL_GET_CCOUNT();
    uint32_t lv = gdo0Level();
    if (now - prev > g_maxSampleGap) g_maxSampleGap = now - prev;
    if (lv != last) {
      if (n < MAX_EDGES) g_edges[n] = {prev, now};
      n++;
      last = lv;
    }
    prev = now;
    if (now - t0 > limit) break;
  }
  return n;
}

// Compara los flancos medidos con las corridas que se mandaron y decodifica lo medido como lo haría
// collar_sub.py decode (cuantizar a bits, buscar 0x30+ID, validar Sum8).
static void report(const t60::Run *runs, int nRuns, int nEdges, int framesSent) {
  const double cpm = getCpuFrequencyMhz();
  Serial.printf("MEAS flancos=%d esperados=%d nivel_inicial=%u muestreo_peor=%.2f µs\n", nEdges, nRuns,
                (unsigned)g_startLevel, g_maxSampleGap / cpm);
  if (nEdges != nRuns || nEdges > MAX_EDGES) {
    Serial.println("MEAS FAIL cantidad de flancos ≠ corridas enviadas");
    return;
  }
  auto mid = [&](int i) { return (g_edges[i].before + (g_edges[i].at - g_edges[i].before) / 2.0) / cpm; };
  auto unc = [&](int i) { return (g_edges[i].at - g_edges[i].before) / cpm / 2.0; };
  const double t0 = mid(0);

  // Acumuladores corrientes (sin vectores O(nRuns): escala a pulsaciones largas sin agotar el heap).
  double ideal = 0, worstErr = 0;
  int outside2 = 0, levelBad = 0;
  double bSum = 0, bSq = 0, bMin = 1e9, bMax = -1e9, prevStart = t0;
  uint32_t bN = 0, frameIdx = 0;
  for (int i = 0; i < nRuns - 1; i++) {
    double d = mid(i + 1) - mid(i);
    ideal += runs[i].tenthsUs / 10.0;
    worstErr = fmax(worstErr, fabs(mid(i + 1) - t0 - ideal));
    // nivel tras el flanco i: lo da la paridad (arranca en bajo → primer flanco sube)
    if ((uint8_t)((i & 1) == 0) != runs[i].level) levelBad++;
    if (runs[i].tenthsUs >= t60::kGapTenthsUs) {  // fin de trama: el flanco siguiente inicia la próxima
      double start = mid(i + 1);
      Serial.printf("MEAS trama %u: período %.3f ms (ideal 126.037)\n", (unsigned)frameIdx++, (start - prevStart) / 1000);
      prevStart = start;
      continue;
    }
    int nb = (int)lround(runs[i].tenthsUs / (double)t60::kBitTenthsUs);
    double b = d / nb;
    bSum += b;
    bSq += b * b;
    bN++;
    bMin = fmin(bMin, b);
    bMax = fmax(bMax, b);
    if (fabs(b - 104.2) > 104.2 * 0.02 && unc(i) + unc(i + 1) < 1.0) outside2++;
  }
  double mean = bN ? bSum / bN : 0, sd = bN ? sqrt(fmax(0, bSq / bN - mean * mean)) : 0;
  Serial.printf("MEAS bit µs: media=%.3f sd=%.3f min=%.2f max=%.2f (ideal 104.200, ±2%% = ±2.084)\n", mean, sd, bMin,
                bMax);
  Serial.printf("MEAS corridas fuera de ±2%% (incertidumbre <1 µs): %d de %u | nivel incorrecto: %d\n", outside2,
                (unsigned)bN, levelBad);
  Serial.printf("MEAS error de flanco vs reloj ideal: max=%.2f µs\n", worstErr);
  double lastEdge = mid(nRuns - 1) - t0;
  double expectLast = 0;
  for (int i = 0; i < nRuns - 1; i++) expectLast += runs[i].tenthsUs / 10.0;
  Serial.printf("MEAS duración total al último flanco: %.3f ms (ideal %.3f) | tramas %d\n", lastEdge / 1000,
                expectLast / 1000, framesSent);

  // Decode de lo medido: bits por redondeo de cada intervalo, separando por silencios > 2 ms.
  std::vector<uint8_t> bits;
  auto decodeChunk = [&]() {
    static const uint8_t head[] = {0, 0, 1, 1, 0, 0, 0, 0};  // 0x30
    for (size_t k = 0; k + 8 + 48 <= bits.size(); k++) {
      if (memcmp(&bits[k], head, 8)) continue;
      uint8_t b[6] = {};
      for (int j = 0; j < 48; j++) b[j / 8] = (b[j / 8] << 1) | bits[k + 8 + j];
      bool ok = (uint8_t)(b[0] + b[1] + b[2] + b[3] + b[4]) == b[5];
      Serial.printf("DECODE id=%02X%02X seq=0x%02X modo=0x%02X potencia=%u chk=0x%02X %s\n", b[0], b[1], b[2],
                    b[3], b[4], b[5], ok ? "OK" : "MAL");
      return;
    }
    if (!bits.empty()) Serial.println("DECODE sin 0x30 tras el preámbulo");
  };
  for (int i = 0; i < nRuns - 1; i++) {
    double d = mid(i + 1) - mid(i);
    uint8_t lv = (i & 1) == 0;
    if (d > 2000) {
      // el silencio arrastra los últimos bits 0 de la trama, si los hay
      for (long j = lround((d - 11000) / 104.2); j > 0; j--) bits.push_back(lv);
      decodeChunk();
      bits.clear();
      continue;
    }
    for (long j = lround(d / 104.2); j > 0; j--) bits.push_back(lv);
  }
  bits.insert(bits.end(), 8, 0);  // la última corrida (silencio final) sigue en bajo: sus bits 0 no tienen flanco
  decodeChunk();
}

// ---------------------------------------------------------------- TX
static uint8_t g_seq;
static t60::Run g_runs[8000];
static std::vector<rmt_symbol_word_t> g_syms;

static bool modeFromName(const char *s, uint8_t &mode) {
  if (!strcmp(s, "luz")) mode = t60::kLuz;
  else if (!strcmp(s, "tono")) mode = t60::kTono;
  else if (!strcmp(s, "vibrar")) mode = t60::kVibrar;
  else if (!strcmp(s, "shock")) mode = t60::kShock;
  else return false;
  mode |= 0x01;  // canal 1
  return true;
}

// Envía las corridas ya construidas en g_runs[0..n). `wdMs` = 0 → watchdog normal (duración + 200 ms);
// >0 → forzado (wdtest). `doReport` → mide en el pad y decodifica lo transmitido. `framesSent` solo informa.
static void transmitRuns(int n, uint32_t wdMs, bool doReport, int framesSent) {
  toSymbols(g_runs, n, g_syms);
  double totalUs = 0;
  for (int i = 0; i < n; i++) totalUs += g_runs[i].tenthsUs / 10.0;

  strobe(SIDLE);
  if (!waitMarc(MARC_IDLE, 50)) { Serial.println("ERR CC1101 no entra a IDLE"); return; }
  if (gdo0Level()) { Serial.println("ERR GDO0 alto antes de TX"); return; }
  strobe(STX);  // FS_AUTOCAL=01 calibra en esta transición
  if (!waitMarc(MARC_TX, 5)) {
    Serial.printf("ERR MARCSTATE=0x%02X tras STX (esperado 0x13)\n", marc());
    strobe(SIDLE);
    return;
  }
  uint8_t marcStart = marc();

  g_wdFired = false;
  wdArm(wdMs ? wdMs : (uint32_t)(totalUs / 1000) + 200);

  rmt_transmit_config_t tc = {};
  tc.flags.eot_level = 0;
  uint32_t t0;
  if (rmt_transmit(g_rmt, g_copy, g_syms.data(), g_syms.size() * sizeof(rmt_symbol_word_t), &tc) != ESP_OK) {
    wdDisarm();
    strobe(SIDLE);
    Serial.println("ERR rmt_transmit");
    return;
  }
  uint32_t captureUs = wdMs ? (wdMs + 60) * 1000 : (uint32_t)totalUs + LEAD_TICKS / 10 - t60::kGapTenthsUs / 10 + 2000;
  int nEdges = capture(captureUs, t0);

  bool done = rmt_tx_wait_all_done(g_rmt, wdMs ? 0 : 50) == ESP_OK;
  uint8_t marcEnd = g_wdFired ? 0xFF : marc();
  wdDisarm();
  if (!done) rmt_disable(g_rmt);
  strobe(SIDLE);
  bool idle = waitMarc(MARC_IDLE, 50);
  uint32_t lvl = gdo0Level();
  if (!done || g_wdFired) { rmt_enable(g_rmt); }

  Serial.printf("TX final: MARCSTATE inicio=0x%02X fin=%s | CC1101 %s, GDO0=%u, watchdog=%s, rmt=%s\n", marcStart,
                g_wdFired ? "(watchdog)" : String(marcEnd, HEX).c_str(), idle ? "IDLE" : "NO-IDLE", (unsigned)lvl,
                g_wdFired ? "DISPARÓ" : "no", done ? "completo" : "cortado");
  if (doReport) report(g_runs, n, nEdges, framesSent);
}

// Una pulsación completa de potencia fija. `wdMs` = 0 → watchdog normal; >0 → forzado (wdtest).
static void transmit(t60::Frame f, int frames, uint32_t wdMs) {
  rfLock();
  int n = t60::pressRuns(f, frames, true, g_runs, sizeof g_runs / sizeof g_runs[0]);
  if (n < 0) { rfUnlock(); Serial.println("ERR no caben las corridas"); return; }
  Serial.printf("TX seq=0x%02X modo=0x%02X potencia=%u tramas=%d+soltar chkXor=0x%02X id=%02X%02X\n", f.seq, f.mode,
                f.power, frames, f.chkXor, f.idHi, f.idLo);
  transmitRuns(n, wdMs, wdMs == 0, frames);
  rfUnlock();
}

static void cmdTx(int argc, char **argv) {
  uint8_t mode;
  if (argc < 3 || !modeFromName(argv[1], mode)) {
    Serial.println("ERR uso: tx <luz|tono|vibrar|shock> <0-99> [tramas 1-6] [badchk|badid]");
    return;
  }
  if ((mode & 0xF0) == t60::kShock && !shockEnabled()) {
    Serial.println(ALLOW_SHOCK ? "ERR shock apagado (interruptor: shock on)" : "ERR shock deshabilitado en compilación (ALLOW_SHOCK=0)");
    return;
  }
  int power = atoi(argv[2]);
  int frames = argc > 3 && isdigit((unsigned char)argv[3][0]) ? atoi(argv[3]) : 1;
  if (power < 0 || power > 99 || frames < 1 || frames > MAX_FRAMES) {
    Serial.printf("ERR potencia 0-99, tramas 1-%d\n", MAX_FRAMES);
    return;
  }
  t60::Frame f;
  f.seq = g_seq;
  f.mode = mode;
  f.power = power;
  for (int i = 3; i < argc; i++) {
    if (!strcmp(argv[i], "badchk")) f.chkXor = 0x5A;
    if (!strcmp(argv[i], "badid")) f.idLo = t60::kIdLo ^ 0x01;  // ID vecino: otro collar
  }
  g_seq += 2;  // tramas con seq, soltar con seq+1
  transmit(f, frames, 0);
}

// Prueba del punto 3: una sola pulsación (mismo seq) con la potencia cambiando entre niveles → ver si el
// collar sigue el cambio dentro del mismo botón. `txramp <modo> <tramas-por-nivel> <p1> [p2 ...]`.
static void cmdTxRamp(int argc, char **argv) {
  uint8_t mode;
  if (argc < 4 || !modeFromName(argv[1], mode)) {
    Serial.println("ERR uso: txramp <luz|tono|vibrar|shock> <tramas-por-nivel> <p1> [p2 ...]");
    return;
  }
  if ((mode & 0xF0) == t60::kShock && !shockEnabled()) {
    Serial.println(ALLOW_SHOCK ? "ERR shock apagado (interruptor: shock on)" : "ERR shock deshabilitado en compilación (ALLOW_SHOCK=0)");
    return;
  }
  int per = atoi(argv[2]);
  if (per < 1 || per > MAX_FRAMES) { Serial.printf("ERR tramas-por-nivel 1-%d\n", MAX_FRAMES); return; }
  uint8_t powers[8];
  int np = 0;
  for (int i = 3; i < argc && np < 8; i++) {
    int p = atoi(argv[i]);
    if (p < 0 || p > 99) { Serial.println("ERR potencia 0-99"); return; }
    powers[np++] = (uint8_t)p;
  }
  if (np < 1) { Serial.println("ERR al menos un nivel de potencia"); return; }
  if (np * per > MAX_FRAMES) {  // acota la pulsación (y g_runs: ~1076 corridas por trama)
    Serial.printf("ERR máximo %d tramas de datos (niveles x tramas-por-nivel)\n", MAX_FRAMES);
    return;
  }
  rfLock();
  int n = t60::rampRuns(g_seq, mode, powers, np, per, true, g_runs, sizeof g_runs / sizeof g_runs[0]);
  if (n < 0) { rfUnlock(); Serial.println("ERR no caben las corridas"); return; }
  Serial.printf("TXRAMP seq=0x%02X modo=0x%02X %dx%d tramas + soltar, potencias:", g_seq, mode, np, per);
  for (int i = 0; i < np; i++) Serial.printf(" %u", powers[i]);
  Serial.println();
  g_seq += 2;
  transmitRuns(n, 0, true, np * per);
  rfUnlock();
}

// Mide la cadencia del collar: manda `count` pulsos discretos (1 trama + soltar, seq nuevo cada uno) separados
// por `gap_ms` de silencio. Daniel cuenta cuántos estímulos distintos siente a cada separación → cada cuánto el
// collar entrega pulsos distinguibles (= piso para la cadencia de la rampa). `txrate <modo> <0-99> <gap_ms> <count>`.
static void cmdTxRate(int argc, char **argv) {
  uint8_t mode;
  if (argc < 5 || !modeFromName(argv[1], mode)) {
    Serial.println("ERR uso: txrate <luz|tono|vibrar|shock> <0-99> <gap_ms 50-2000> <count 1-10>");
    return;
  }
  if ((mode & 0xF0) == t60::kShock && !shockEnabled()) {
    Serial.println(ALLOW_SHOCK ? "ERR shock apagado (interruptor: shock on)" : "ERR shock deshabilitado en compilación (ALLOW_SHOCK=0)");
    return;
  }
  int power = atoi(argv[2]), gap = atoi(argv[3]), count = atoi(argv[4]);
  if (power < 0 || power > 99 || gap < 50 || gap > 2000 || count < 1 || count > 10) {
    Serial.println("ERR potencia 0-99, gap 50-2000 ms, count 1-10");
    return;
  }
  Serial.printf("TXRATE %d pulsos de %s %d, gap %d ms (pulso ≈241 ms → período ≈%d ms)\n", count, argv[1], power, gap,
                241 + gap);
  for (int i = 0; i < count; i++) {
    t60::Frame f;
    f.seq = g_seq;
    f.mode = mode;
    f.power = power;
    g_seq += 2;
    rfLock();
    int n = t60::pressRuns(f, 1, true, g_runs, sizeof g_runs / sizeof g_runs[0]);
    if (n >= 0) transmitRuns(n, 0, false, 1);
    rfUnlock();
    if (n < 0) { Serial.println("ERR no caben las corridas"); return; }
    Serial.printf("  pulso %d/%d (seq=0x%02X) enviado\n", i + 1, count, f.seq);
    if (i + 1 < count) delay(gap);
  }
  Serial.println("TXRATE fin");
}

// Re-disparo APRETADO: `count` pulsos de UNA sola trama (seq nuevo cada uno → el collar lo toma como press nuevo),
// separados por `gap_ms` (puede ser 0). Período = 126 ms (trama) + gap → mucho más corto que txrate (241 ms + gap).
// Para buscar el punto en que el siguiente pulso entra justo al terminar el del collar y se sienta continuo.
// Soltar una sola vez al final. `txtight <modo> <0-99> <gap_ms 0-500> <count 1-20>`.
static void cmdTxTight(int argc, char **argv) {
  uint8_t mode;
  if (argc < 5 || !modeFromName(argv[1], mode)) {
    Serial.println("ERR uso: txtight <luz|tono|vibrar|shock> <0-99> <gap_ms 0-500> <count 1-20>");
    return;
  }
  if ((mode & 0xF0) == t60::kShock && !shockEnabled()) {
    Serial.println(ALLOW_SHOCK ? "ERR shock apagado (interruptor: shock on)" : "ERR shock deshabilitado en compilación (ALLOW_SHOCK=0)");
    return;
  }
  int power = atoi(argv[2]), gap = atoi(argv[3]), count = atoi(argv[4]);
  if (power < 0 || power > 99 || gap < 0 || gap > 500 || count < 1 || count > 20) {
    Serial.println("ERR potencia 0-99, gap 0-500 ms, count 1-20");
    return;
  }
  Serial.printf("TXTIGHT %d pulsos de 1 trama de %s %d, gap %d ms (período ≈%d ms) + soltar\n", count, argv[1], power,
                gap, 126 + gap);
  for (int i = 0; i < count; i++) {
    t60::Frame f;
    f.seq = g_seq;
    f.mode = mode;
    f.power = power;
    g_seq += 2;
    bool last = (i == count - 1);
    rfLock();
    int n = t60::pressRuns(f, 1, last, g_runs, sizeof g_runs / sizeof g_runs[0]);  // soltar solo en el último
    if (n >= 0) transmitRuns(n, 0, false, 1);
    rfUnlock();
    if (n < 0) { Serial.println("ERR no caben las corridas"); return; }
    if (!last && gap > 0) delay(gap);
  }
  Serial.println("TXTIGHT fin");
}

static void cmdWdTest() {
  // Trama de tono 0 cortada a los 40 ms: solo sale preámbulo (109 ms), nunca llega a sync → el collar no la toma.
  t60::Frame f;
  f.seq = g_seq;
  f.mode = t60::kTono | 1;
  transmit(f, 1, 40);
  Serial.printf("WDTEST %s (callback ejecutado %u veces)\n",
                g_wdFired && marc() == MARC_IDLE && gdo0Level() == 0 ? "PASS" : "FAIL", (unsigned)g_wdRuns);
}

static void cmdRegs() {
  uint8_t pn = readStatus(PARTNUM), ver = readStatus(VERSION);
  Serial.printf("PARTNUM=0x%02X (esperado 0x00) VERSION=0x%02X (esperado 0x14) MARCSTATE=0x%02X\n", pn, ver, marc());
  uint8_t expect[0x2F];
  memcpy(expect, RESET_VALUES, sizeof expect);
  for (auto &r : g_cfg) expect[r[0]] = r[1];
  int diffs = 0;
  uint8_t got[0x2F];
  for (uint8_t a = 0; a < 0x2F; a++) {
    got[a] = readReg(a);
    // Resultados de calibración: FSCAL3[3:0], FSCAL2[4:0], FSCAL1 los escribe el chip (§29.2, págs. 89–90).
    bool cal = a == FSCAL2 || a == FSCAL1 || a == FSCAL3;
    // FOCCFG[7:6] son "R0 Not used" (pág. 85) pero el chip lee 0x76 en reset, no 0x36 → comparar solo [5:0].
    uint8_t mask = a == 0x19 ? 0x3F : 0xFF;
    bool ok = cal ? (a != FSCAL3 || (got[a] & 0xF0) == (expect[a] & 0xF0)) : (got[a] & mask) == (expect[a] & mask);
    if (!ok) diffs++;
    Serial.printf("  0x%02X %-8s 0x%02X esperado 0x%02X %s%s\n", a, REG_NAMES[a], got[a], expect[a],
                  ok ? "OK" : "DIFERENTE", cal ? " (calibración)" : "");
  }
  uint8_t pa = readPa0();
  if (pa != PA_433_10DBM) diffs++;
  Serial.printf("  PATABLE[0] 0x%02X esperado 0x%02X %s\n", pa, PA_433_10DBM, pa == PA_433_10DBM ? "OK" : "DIFERENTE");
  uint32_t fw = (got[FREQ2] << 16) | (got[FREQ1] << 8) | got[FREQ0];
  double dr = (256.0 + got[MDMCFG3]) * pow(2, got[MDMCFG4] & 0x0F) / pow(2, 28) * XTAL_HZ;
  double dev = XTAL_HZ / pow(2, 17) * (8 + (got[DEVIATN] & 7)) * pow(2, (got[DEVIATN] >> 4) & 7);
  Serial.printf("  → f=%.0f Hz, tasa=%.1f Bd (muestreo GDO0 cada %.1f µs), desviación=%.0f Hz, modulación=%s\n",
                fw * XTAL_HZ / 65536.0, dr, 1e6 / (8 * dr), dev, ((got[MDMCFG2] >> 4) & 7) == 0 ? "2-FSK" : "OTRA");
  Serial.printf("REGS %s (%d diferencias) | RMT %s | ALLOW_SHOCK=%d shock=%s\n",
                diffs == 0 && pn == 0x00 && ver == 0x14 ? "OK" : "FAIL", diffs, g_rmtDma ? "DMA" : "sin DMA",
                ALLOW_SHOCK, shockEnabled() ? "ON" : "off");
}

// ---------------------------------------------------------------- punto 2: captura de GPIO del encoder/KEY
// Registra las transiciones de GPIO 4/5/0/6 con timestamp (ciclos de CPU) para medir rebote, transiciones por
// detent, velocidad de giro e intervalos de doble click. No transmite nada; es solo entrada. Reusa el patrón
// de capture() (muestreo cerrado de GPIO_IN_REG), pero guarda el nibble de los 4 pines en vez de un solo bit.
struct EncT {
  uint32_t cyc;
  uint8_t nib;  // bit0=A bit1=B bit2=BTN bit3=KEY
};
static EncT *g_enc;
static constexpr int MAX_ENC = 6000;

static inline uint8_t encNib() {
  uint32_t r = REG_READ(GPIO_IN_REG);
  return ((r >> PIN_ENC_A) & 1) | (((r >> PIN_ENC_B) & 1) << 1) | (((r >> PIN_ENC_BTN) & 1) << 2) |
         (((r >> PIN_KEY) & 1) << 3);
}

static void cmdCap(int argc, char **argv) {
  uint32_t secs = argc > 1 ? strtoul(argv[1], nullptr, 10) : 6;
  if (secs < 1) secs = 1;
  if (secs > 15) secs = 15;  // CCOUNT (32 bits) envuelve a ~17.9 s @240 MHz → tope 15 s
  const double cpm = getCpuFrequencyMhz();
  Serial.printf("CAP %u s — hacé el gesto ahora. Pines activos en bajo: A=%d B=%d BTN=%d KEY=%d\n", secs,
                PIN_ENC_A, PIN_ENC_B, PIN_ENC_BTN, PIN_KEY);
  disableLoopWDT();
  uint32_t n = 0, t0 = XTHAL_GET_CCOUNT();
  uint8_t last = encNib();
  g_enc[n++] = {t0, last};
  uint32_t limit = (uint32_t)(secs * 1000000.0 * cpm);
  for (;;) {
    uint32_t now = XTHAL_GET_CCOUNT();
    uint8_t v = encNib();
    if (v != last) {
      if (n < MAX_ENC) g_enc[n] = {now, v};
      n++;
      last = v;
    }
    if (now - t0 > limit) break;
  }
  enableLoopWDT();
  uint32_t m = n < MAX_ENC ? n : MAX_ENC;
  Serial.printf("CAP %u transiciones%s\n", (unsigned)n, n > MAX_ENC ? " (TRUNCADO)" : "");
  for (uint32_t i = 0; i < m; i++) {
    uint8_t x = g_enc[i].nib;
    Serial.printf("%.1f A%u B%u BTN%u KEY%u\n", (g_enc[i].cyc - t0) / cpm, x & 1, (x >> 1) & 1, (x >> 2) & 1,
                  (x >> 3) & 1);
  }
  Serial.println("CAP fin");
}

static void cmdLog();    // definidos en la sección de gestos (fase 5), más abajo
static void cmdGstat();
static void cmdSet(int argc, char **argv);
static void beep(int freq, int ms);
static void cmdLcd();
static void cmdMenu(int argc, char **argv);
static void cmdNvs(int argc, char **argv);
static void cmdShock(int argc, char **argv);
static void pollCombo(uint32_t now);

static void handle(char *line) {
  char *argv[8];
  int argc = 0;
  for (char *tok = strtok(line, " \t"); tok && argc < 8; tok = strtok(nullptr, " \t")) argv[argc++] = tok;
  if (!argc) return;
  if (!strcmp(argv[0], "ping")) Serial.printf("PONG control-t60 0x%02X 0x%02X\n", readStatus(PARTNUM), readStatus(VERSION));
  else if (!strcmp(argv[0], "regs")) cmdRegs();
  else if (!strcmp(argv[0], "tx")) cmdTx(argc, argv);
  else if (!strcmp(argv[0], "txramp")) cmdTxRamp(argc, argv);
  else if (!strcmp(argv[0], "txrate")) cmdTxRate(argc, argv);
  else if (!strcmp(argv[0], "txtight")) cmdTxTight(argc, argv);
  else if (!strcmp(argv[0], "wdtest")) cmdWdTest();
  else if (!strcmp(argv[0], "cap")) cmdCap(argc, argv);
  else if (!strcmp(argv[0], "log")) cmdLog();
  else if (!strcmp(argv[0], "gstat")) cmdGstat();
  else if (!strcmp(argv[0], "set")) cmdSet(argc, argv);
  else if (!strcmp(argv[0], "lcd")) cmdLcd();
  else if (!strcmp(argv[0], "menu")) cmdMenu(argc, argv);
  else if (!strcmp(argv[0], "nvs")) cmdNvs(argc, argv);
  else if (!strcmp(argv[0], "shock")) cmdShock(argc, argv);
  else if (!strcmp(argv[0], "beep")) { beep(argc > 1 ? atoi(argv[1]) : 1800, argc > 2 ? atoi(argv[2]) : 150); Serial.println("BEEP"); }
  else Serial.println("ERR comandos: ping regs tx txramp txrate txtight wdtest cap log gstat set lcd menu nvs shock beep");
}

// ================================================================ Fase 5: control por gestos
using gesture::Act;
using gesture::Action;
using gesture::In;
using gesture::Mode;

static gesture::Machine g_m;
static gesture::Config g_cfg5;  // config viva, ajustable por serial con `set` (para afinar sin reflashear)
static Action g_actBuf[16];

// Último estímulo decidido (para la pantalla): modo pedido por el gesto, modo enviado (shock→vibrar) y potencia.
static uint32_t g_lastN;
static Mode g_lastReq = Mode::None, g_lastSent = Mode::None;
static uint8_t g_lastPow;

static uint8_t collarMode(Mode m) {  // gesture::Mode → byte de modo del collar (canal 1)
  switch (m) {
    case Mode::Shock: return t60::kShock | 1;
    case Mode::Vibrar: return t60::kVibrar | 1;
    case Mode::Tono: return t60::kTono | 1;
    case Mode::Luz: return t60::kLuz | 1;
    default: return t60::kVibrar | 1;
  }
}

// --- registro de eventos (entrada → decisión → trama), volcable con `log` ---
struct LogE { uint32_t t; char kind; uint8_t a, b, c; };
static LogE g_log[256];
static volatile uint32_t g_logN;
static portMUX_TYPE g_logMux = portMUX_INITIALIZER_UNLOCKED;
static void logAdd(char kind, uint8_t a, uint8_t b, uint8_t c) {
  portENTER_CRITICAL(&g_logMux);
  g_log[g_logN & 255] = {micros(), kind, a, b, c};
  g_logN++;
  portEXIT_CRITICAL(&g_logMux);
}

// --- tarea de TX: consume una cola de órdenes y transmite trama por trama; sostiene el tono a 126 ms ---
struct TxCmd { uint8_t mode; uint8_t power; };  // cada orden = un pulso discreto (1 trama + soltar)
static QueueHandle_t g_txQueue;
static uint8_t g_txSeq;

// Diagnóstico de la rampa (2026-10-20: tramas a ~2.27 s fijos en vez de ~0.24 s): registra en `log` una 'T' por
// envío con la espera del lock y la duración de la TX en unidades de 10 ms, y si actuó el watchdog.
static void sendFrame(uint8_t seq, uint8_t mode, uint8_t power, bool release) {
  uint32_t t0 = micros();
  rfLock();
  uint32_t t1 = micros();
  t60::Frame f;
  f.seq = seq;
  f.mode = mode;
  f.power = power;
  int n = t60::pressRuns(f, 1, release, g_runs, sizeof g_runs / sizeof g_runs[0]);
  if (n > 0) transmitRuns(n, 0, false, 1);
  uint32_t t2 = micros();
  rfUnlock();
  auto u10 = [](uint32_t us) { return (uint8_t)(us / 10000 > 255 ? 255 : us / 10000); };
  logAdd('T', u10(t1 - t0), u10(t2 - t1), g_wdFired);
}
// La tarea de TX solo transmite pulsos discretos (1 trama + soltar), seq nuevo cada uno. En este collar no
// existe el sostenido (refractario ~1 s): tanto el tono como la rampa son pulsos repetidos que arma la máquina.
static void txTask(void *) {
  for (;;) {
    TxCmd c;
    if (xQueueReceive(g_txQueue, &c, portMAX_DELAY) == pdTRUE) {
      sendFrame(g_txSeq, c.mode, c.power, true);  // trama + soltar
      logAdd('F', c.mode, c.power, g_txSeq);
      g_txSeq += 2;
    }
  }
}

// Acciones de la máquina → órdenes de TX. Cada pulso viene como par Stim+Release (pulso discreto). En fase de
// Con el interruptor de shock apagado (o ALLOW_SHOCK=0), shock→vibrar (gesture::effectiveMode).
static void dispatch(Action *a, int n) {
  for (int i = 0; i < n; i++) {
    if (a[i].act != Act::Stim) continue;  // el Release del par no genera trama aparte (pressRuns ya la agrega)
    Mode gm = gesture::effectiveMode(a[i].mode, shockEnabled());
    uint8_t mode = collarMode(gm), power = a[i].power;
    logAdd('D', 0, mode, power);
    g_lastReq = a[i].mode;
    g_lastSent = gm;
    g_lastPow = power;
    g_lastN++;
    TxCmd c{mode, power};
    xQueueSend(g_txQueue, &c, 0);
    if (i + 1 < n && a[i + 1].act == Act::Release) i++;
  }
}

// --- encoder por PCNT del S3 (cuadratura con filtro de glitches) ---
static pcnt_unit_handle_t g_pcnt;
// MEDIDO con `cap 10` (2026-10-20, 10 clicks → 20 transiciones limpias): la rueda descansa en A1B1 y en A0B0
// alternadamente (11→01→00 un click, 00→10→11 el siguiente) → 2 cuentas x4 por click. Con 4 hacían falta 2 clicks.
static constexpr int COUNTS_PER_DETENT = 2;
static int g_lastCount;

static void encoderInit() {
  pcnt_unit_config_t uc = {};
  uc.low_limit = -1000;
  uc.high_limit = 1000;
  if (pcnt_new_unit(&uc, &g_pcnt) != ESP_OK) { g_pcnt = nullptr; return; }
  pcnt_glitch_filter_config_t gf = {};
  // Rebote medido con `cap`: hasta 11.6 µs; la transición legítima más rápida, 23 ms. 12 µs ≈ máximo del filtro
  // del PCNT (1023 ciclos de APB a 80 MHz = 12.8 µs).
  gf.max_glitch_ns = 12000;
  pcnt_unit_set_glitch_filter(g_pcnt, &gf);
  pcnt_chan_config_t ca = {};
  ca.edge_gpio_num = PIN_ENC_A;
  ca.level_gpio_num = PIN_ENC_B;
  pcnt_channel_handle_t chA;
  pcnt_new_channel(g_pcnt, &ca, &chA);
  pcnt_channel_set_edge_action(chA, PCNT_CHANNEL_EDGE_ACTION_DECREASE, PCNT_CHANNEL_EDGE_ACTION_INCREASE);
  pcnt_channel_set_level_action(chA, PCNT_CHANNEL_LEVEL_ACTION_KEEP, PCNT_CHANNEL_LEVEL_ACTION_INVERSE);
  pcnt_chan_config_t cb = {};
  cb.edge_gpio_num = PIN_ENC_B;
  cb.level_gpio_num = PIN_ENC_A;
  pcnt_channel_handle_t chB;
  pcnt_new_channel(g_pcnt, &cb, &chB);
  pcnt_channel_set_edge_action(chB, PCNT_CHANNEL_EDGE_ACTION_INCREASE, PCNT_CHANNEL_EDGE_ACTION_DECREASE);
  pcnt_channel_set_level_action(chB, PCNT_CHANNEL_LEVEL_ACTION_KEEP, PCNT_CHANNEL_LEVEL_ACTION_INVERSE);
  pcnt_unit_enable(g_pcnt);
  pcnt_unit_clear_count(g_pcnt);
  pcnt_unit_start(g_pcnt);
}

// --- modo menú (fase 3): con el menú abierto, encoder y botones van a LVGL y NO a la máquina de gestos ---
static bool g_inMenu;
static int32_t g_menuDiff;  // muescas pendientes para LVGL (+ = derecha = siguiente/subir)
static lv_group_t *g_grp;   // grupo de foco del menú (se crea en uiInit)
static void menuButton(int pin, bool down);

// Menú: editando un valor, girar rápido salta de a varios (cambiar 5→25 sin 20 clicks). Navegar entre filas no
// acelera (1 click = 1 fila). Umbrales = tiempo entre clicks.
static int menuStep(uint32_t now) {
  static uint32_t last;
  uint32_t dt = now - last;
  last = now;
  if (!g_grp || !lv_group_get_editing(g_grp)) return 1;
  return dt < 60000 ? 4 : dt < 120000 ? 2 : 1;
}

static void pollEncoder(uint32_t now) {
  if (!g_pcnt) return;
  int cnt = 0;
  pcnt_unit_get_count(g_pcnt, &cnt);
  int delta = cnt - g_lastCount;
  while (delta >= COUNTS_PER_DETENT) {
    g_lastCount += COUNTS_PER_DETENT;
    delta -= COUNTS_PER_DETENT;
    if (g_inMenu) { g_menuDiff += menuStep(now); continue; }
    logAdd('I', (uint8_t)In::Wheel, 1, 0);
    dispatch(g_actBuf, g_m.feed(In::Wheel, +1, now, g_actBuf, 16));
  }
  while (delta <= -COUNTS_PER_DETENT) {
    g_lastCount -= COUNTS_PER_DETENT;
    delta += COUNTS_PER_DETENT;
    if (g_inMenu) { g_menuDiff -= menuStep(now); continue; }
    logAdd('I', (uint8_t)In::Wheel, 0xFF, 0);
    dispatch(g_actBuf, g_m.feed(In::Wheel, -1, now, g_actBuf, 16));
  }
}

// --- botones con debounce (activos en bajo) ---
struct Btn { int pin; bool pressed; bool lastRaw; uint32_t tEdge; };
static Btn g_btnEnc{PIN_ENC_BTN, false, true, 0}, g_btnKey{PIN_KEY, false, true, 0};

static void pollBtn(Btn &b, In downEv, In upEv, uint32_t now) {
  bool raw = digitalRead(b.pin) == LOW;  // apretado = LOW
  if (raw != b.lastRaw) { b.lastRaw = raw; b.tEdge = now; }
  if (b.pressed != raw && (uint32_t)(now - b.tEdge) >= g_m.config().debounceUs) {
    b.pressed = raw;
    if (g_inMenu) { menuButton(b.pin, raw); return; }
    In ev = raw ? downEv : upEv;
    logAdd('I', (uint8_t)ev, 0, 0);
    dispatch(g_actBuf, g_m.feed(ev, 0, now, g_actBuf, 16));
  }
}

// --- feedback: el primer LED WS2812 (GPIO14). verde=armado, rojo=pulsación, apagado=desarmado ---
static void updateLed() {
  static int last = -1;
  int st = !g_m.armed() ? 0 : (g_m.pressActive() ? 2 : 1);
  if (st == last) return;
  last = st;
  if (st == 0) rgbLedWrite(14, 0, 0, 0);
  else if (st == 1) rgbLedWrite(14, 0, 6, 0);
  else rgbLedWrite(14, 16, 0, 0);
}

// --- audio local: parlante del T-Embed por I2S (MAX98357A). Esquemático: BCLK=46, LRCLK=40, DIN=7. ---
// Se usa SOLO para feedback en el dispositivo (confirmar armado/desarme sin mirar), NUNCA es un estímulo al collar.
static i2s_chan_handle_t g_i2s;
static void audioInit() {
  i2s_chan_config_t cc = I2S_CHANNEL_DEFAULT_CONFIG(I2S_NUM_0, I2S_ROLE_MASTER);
  if (i2s_new_channel(&cc, &g_i2s, nullptr) != ESP_OK) { g_i2s = nullptr; return; }
  i2s_std_config_t sc = {};
  sc.clk_cfg = I2S_STD_CLK_DEFAULT_CONFIG(16000);
  sc.slot_cfg = I2S_STD_PHILIPS_SLOT_DEFAULT_CONFIG(I2S_DATA_BIT_WIDTH_16BIT, I2S_SLOT_MODE_MONO);
  sc.gpio_cfg.mclk = I2S_GPIO_UNUSED;
  sc.gpio_cfg.bclk = (gpio_num_t)46;
  sc.gpio_cfg.ws = (gpio_num_t)40;
  sc.gpio_cfg.dout = (gpio_num_t)7;
  sc.gpio_cfg.din = I2S_GPIO_UNUSED;
  if (i2s_channel_init_std_mode(g_i2s, &sc) != ESP_OK) { g_i2s = nullptr; return; }
  // NO se habilita aquí: el canal se habilita solo durante cada beep y se deshabilita al final, si no el DMA
  // repite el último buffer y el tono no para (bug observado 2026-10-20).
}
static void beep(int freq, int ms) {
  if (!g_i2s) return;
  const int sr = 16000;
  int total = sr * ms / 1000, half = freq > 0 ? sr / (2 * freq) : 1;
  int16_t buf[256];
  int idx = 0;
  size_t wr;
  i2s_channel_enable(g_i2s);
  for (int i = 0; i < total; i++) {
    buf[idx++] = ((i / half) & 1) ? 7000 : -7000;  // onda cuadrada
    if (idx == 256) { i2s_channel_write(g_i2s, buf, sizeof buf, &wr, 100); idx = 0; }
  }
  if (idx) i2s_channel_write(g_i2s, buf, idx * sizeof(int16_t), &wr, 100);
  int16_t zero[256] = {0};  // vaciar el DMA con silencio para que no repita el tono, y detener el canal
  for (int i = 0; i < 4; i++) i2s_channel_write(g_i2s, zero, sizeof zero, &wr, 100);
  i2s_channel_disable(g_i2s);
}

static void pollGestures() {
  static bool prevActive = false;
  static bool prevArmed = false;
  uint32_t now = micros();
  pollEncoder(now);
  pollBtn(g_btnEnc, In::BtnDown, In::BtnUp, now);
  pollBtn(g_btnKey, In::KeyDown, In::KeyUp, now);
  pollCombo(now);  // KEY + botón mantenidos (desarmado) → enciende la pantalla y abre el menú
  dispatch(g_actBuf, g_m.tick(now, g_actBuf, 16));
  bool active = g_m.pressActive();
  if (prevActive && !active) xQueueReset(g_txQueue);  // dejó de girar / soltó KEY → no mandar pulsos encolados
  prevActive = active;
  bool armedNow = g_m.armed();
  if (armedNow != prevArmed) {  // confirmación LOCAL (no al collar): armar = beep agudo, desarmar = grave
    beep(armedNow ? 1800 : 500, armedNow ? 120 : 200);
    prevArmed = armedNow;
  }
  updateLed();
}

// ================================================================ Fase 3: pantalla (LVGL) + ajustes + NVS
// La pantalla vive APAGADA (backlight 21 en bajo + ST7789 en SLPIN) para ahorrar batería. Se enciende SOLO con la
// combinación KEY + botón del encoder mantenidos juntos COMBO_US estando DESARMADO, y abre directo el menú de
// ajustes. Con el menú abierto (g_inMenu) el encoder y los botones van a LVGL, no a la máquina de gestos (que no
// puede armarse porque no recibe eventos). Salir: "Guardar" (NVS + aplica), "Salir"/KEY/inactividad (descarta).
// Al salir SIEMPRE g_m.reset(): limpia los KeyDown/BtnDown de la combinación cuyo Up nunca le llegó
// (test_menu_exit_reset_clears_held_keys).
// SPI: LVGL corre solo en loop() (core 1) y cada lv_timer_handler()/lv_refr_now() va dentro de lcdLock():
// rfLock con espera 0 (si la tarea de TX tiene el RF se salta la vuelta → nunca se dibuja con TX en curso) y
// después g_spiLock (el de cada acceso al CC1101). El flush asume que el que llama ya tiene el lock.
static LcdT60 g_lcd;
static bool g_lcdOk;
static constexpr uint32_t COMBO_US = 1500000;    // KEY + botón del encoder mantenidos 1.5 s
static constexpr uint32_t MENU_IDLE_MS = 60000;  // 60 s sin tocar nada → cierra sin guardar y apaga
static bool g_menuMask;           // tras abrir: ignorar entradas hasta soltar los dos botones de la combinación
static bool g_menuBtn;            // estado del botón del encoder que ve LVGL
static uint32_t g_menuActivity;   // millis() de la última entrada en el menú
enum MenuReq : uint8_t { REQ_NONE, REQ_SAVE, REQ_EXIT };
static MenuReq g_menuReq;         // lo piden los callbacks de LVGL; se atiende fuera de lv_timer_handler (lock)
static gesture::Config g_edit;    // copia que edita el menú; solo se aplica al guardar
static int g_nvsFound = -1;       // campos leídos de la NVS al arrancar (para `nvs`)
static int32_t g_lightMode = 1;   // modo de luz elegido (1-3, se guarda con "Guardar"); 1 = el único verificado
static bool g_lightOn;            // último estado ORDENADO (el collar no informa el suyo); arranca apagado

static const char *modeName(Mode m) {
  switch (m) {
    case Mode::Shock: return "shock";
    case Mode::Vibrar: return "vibrar";
    case Mode::Tono: return "tono";
    case Mode::Luz: return "luz";
    default: return "-";
  }
}

static bool lcdLock() {
  if (xSemaphoreTake(g_rfMutex, 0) != pdTRUE) return false;
  xSemaphoreTake(g_spiLock, portMAX_DELAY);
  return true;
}
static void lcdLockWait() {  // abrir/cerrar el menú: esperar a que termine un pulso en curso (≤ ~250 ms)
  rfLock();
  xSemaphoreTake(g_spiLock, portMAX_DELAY);
}
static void lcdUnlock() {
  xSemaphoreGive(g_spiLock);
  rfUnlock();
}

// --- NVS (Preferences) detrás de settings::Store ---
struct NvsStore : settings::Store {
  Preferences p;
  bool ok;
  explicit NvsStore(bool readOnly) { ok = p.begin("t60", readOnly); }
  ~NvsStore() override { if (ok) p.end(); }
  bool get(const char *k, int32_t &v) override {
    if (!ok || !p.isKey(k)) return false;
    v = p.getInt(k);
    return true;
  }
  bool put(const char *k, int32_t v) override { return ok && p.putInt(k, v) == sizeof(int32_t); }
};

// --- LVGL: display (flush a LovyanGFX) + encoder ---
static lv_display_t *g_disp;
static uint8_t g_lvBuf[320 * 24 * 2];  // buffer parcial: 24 líneas
static lv_obj_t *g_sl[settings::kCount], *g_val[settings::kCount], *g_hdr, *g_hdrState, *g_hdrInfo;
static lv_obj_t *g_lightSw, *g_lightDd, *g_noteShock;

static void lvFlush(lv_display_t *d, const lv_area_t *a, uint8_t *px) {
  uint32_t w = lv_area_get_width(a), h = lv_area_get_height(a);
  g_lcd.startWrite();
  g_lcd.setAddrWindow(a->x1, a->y1, w, h);
  g_lcd.writePixels((const uint16_t *)px, w * h, true);  // LVGL RGB565 little-endian → el panel lo quiere big-endian
  g_lcd.endWrite();
  lv_display_flush_ready(d);
}

static void lvEncRead(lv_indev_t *, lv_indev_data_t *d) {
  d->enc_diff = (int16_t)g_menuDiff;
  g_menuDiff = 0;
  d->state = g_menuBtn ? LV_INDEV_STATE_PRESSED : LV_INDEV_STATE_RELEASED;
}

static void fmtValue(int i, int32_t v, char *buf, size_t n) {
  switch (settings::field(i).unit) {
    case settings::Unit::Ms: snprintf(buf, n, "%ld ms", (long)v); break;
    case settings::Unit::Count: snprintf(buf, n, "%ld rev", (long)v); break;
    case settings::Unit::Clicks: snprintf(buf, n, "%ld clk", (long)v); break;
    case settings::Unit::Sec: snprintf(buf, n, "%ld s", (long)v); break;
    case settings::Unit::Bool: snprintf(buf, n, "%s", v ? "SI" : "NO"); break;
    default: snprintf(buf, n, "%ld", (long)v); break;
  }
}

// Vuelca g_edit (ya normalizada) a todos los sliders y valores: bajar el TOPE arrastra visiblemente BAJO/ALTO/etc.
static void uiLoadValues() {
  char buf[16];
  for (int i = 0; i < settings::kCount; i++) {
    const auto &f = settings::field(i);
    int32_t v = settings::get(g_edit, i);
    if (f.unit == settings::Unit::Bool) {
      if (v) lv_obj_add_state(g_sl[i], LV_STATE_CHECKED);
      else lv_obj_remove_state(g_sl[i], LV_STATE_CHECKED);
    } else {
      lv_slider_set_value(g_sl[i], (v - f.min) / f.step, LV_ANIM_OFF);
    }
    fmtValue(i, v, buf, sizeof buf);
    lv_label_set_text(g_val[i], buf);
  }
}

static void uiUpdateHeader() {
  static int lastArmed = -1, lastShock = -1;
  static uint32_t lastN = UINT32_MAX;
  int armed = g_m.armed(), shock = shockEnabled();
  if (armed == lastArmed && lastN == g_lastN && shock == lastShock) return;
  lastArmed = armed;
  lastN = g_lastN;
  lastShock = shock;
  // el estado del shock solo se MUESTRA aquí; se cambia por serial (`shock on|off`)
  lv_label_set_text(g_hdrState, armed ? (shock ? "ARMADO  SHOCK" : "ARMADO") : (shock ? "DESARMADO  SHOCK" : "DESARMADO"));
  lv_obj_set_style_bg_color(g_hdr, armed ? lv_palette_darken(LV_PALETTE_RED, 2)
                                         : shock ? lv_palette_darken(LV_PALETTE_DEEP_ORANGE, 3) : lv_color_hex(0x30343A), 0);
  lv_label_set_text(g_noteShock, shock ? "shock ENCENDIDO (se apaga por serial: shock off)"
                                       : "shock apagado: BAJO/ALTO salen como vibrar");
  lv_obj_set_style_text_color(g_noteShock, shock ? lv_palette_main(LV_PALETTE_DEEP_ORANGE) : lv_color_hex(0x4DD0E1), 0);
  char buf[64];
  if (g_lastReq == Mode::None) snprintf(buf, sizeof buf, "rampa %u/%u  ult: -", g_m.rampLevel(), g_m.config().cap);
  else if (g_lastReq != g_lastSent)
    snprintf(buf, sizeof buf, "rampa %u/%u  ult: %s>%s %u", g_m.rampLevel(), g_m.config().cap, modeName(g_lastReq),
             modeName(g_lastSent), g_lastPow);
  else snprintf(buf, sizeof buf, "rampa %u/%u  ult: %s %u", g_m.rampLevel(), g_m.config().cap, modeName(g_lastSent),
                g_lastPow);
  lv_label_set_text(g_hdrInfo, buf);
}

static void onSlider(lv_event_t *e) {
  int i = (int)(intptr_t)lv_event_get_user_data(e);
  lv_obj_t *sl = (lv_obj_t *)lv_event_get_target(e);
  const auto &f = settings::field(i);
  if (f.unit == settings::Unit::Bool) settings::set(g_edit, i, lv_obj_has_state(sl, LV_STATE_CHECKED) ? 1 : 0);
  else settings::set(g_edit, i, f.min + lv_slider_get_value(sl) * f.step);
  settings::normalize(g_edit);
  uiLoadValues();
}
static void onFocus(lv_event_t *e) {  // resaltar la fila completa del campo enfocado
  lv_obj_t *row = lv_obj_get_parent((lv_obj_t *)lv_event_get_target(e));
  if (lv_event_get_code(e) == LV_EVENT_FOCUSED) lv_obj_add_state(row, LV_STATE_CHECKED);
  else lv_obj_remove_state(row, LV_STATE_CHECKED);
}
// Luz del collar: transmite YA (por la cola de la tarea de TX, igual que un gesto). No es un estímulo.
static void lightSend(uint8_t power) {
  TxCmd c{(uint8_t)(t60::kLuz | 1), power};
  logAdd('L', 0, c.mode, power);
  xQueueSend(g_txQueue, &c, 0);
  Serial.printf("LUZ %s\n", power ? String(power).c_str() : "apagar");
}
static void onLightSw(lv_event_t *e) {
  g_lightOn = lv_obj_has_state((lv_obj_t *)lv_event_get_target(e), LV_STATE_CHECKED);
  lightSend(g_lightOn ? (uint8_t)g_lightMode : 0);
}
static void onLightDd(lv_event_t *e) {
  g_lightMode = settings::clampLightMode(lv_dropdown_get_selected((lv_obj_t *)lv_event_get_target(e)) + 1);
  if (g_lightOn) lightSend((uint8_t)g_lightMode);  // apagada: solo elige el modo para la próxima vez
}
static void onSave(lv_event_t *) { g_menuReq = REQ_SAVE; }
static void onExit(lv_event_t *) { g_menuReq = REQ_EXIT; }
static void onDefaults(lv_event_t *) {
  g_edit = gesture::defaults();
  settings::normalize(g_edit);
  uiLoadValues();
}

static lv_obj_t *mkButton(lv_obj_t *parent, const char *txt, lv_color_t col, lv_event_cb_t cb) {
  lv_obj_t *b = lv_button_create(parent);
  lv_obj_set_height(b, 30);
  lv_obj_set_flex_grow(b, 1);
  lv_obj_set_style_bg_color(b, col, 0);
  lv_obj_add_flag(b, LV_OBJ_FLAG_SCROLL_ON_FOCUS);
  lv_obj_add_event_cb(b, cb, LV_EVENT_CLICKED, nullptr);
  lv_obj_t *l = lv_label_create(b);
  lv_label_set_text(l, txt);
  lv_obj_center(l);
  lv_group_add_obj(g_grp, b);
  return b;
}

static void uiBuild() {
  lv_obj_t *scr = lv_screen_active();
  lv_obj_set_style_bg_color(scr, lv_color_hex(0x101214), 0);
  lv_obj_remove_flag(scr, LV_OBJ_FLAG_SCROLLABLE);

  // encabezado = pantalla de estado compacta (ARMADO/DESARMADO, rampa, último gesto)
  g_hdr = lv_obj_create(scr);
  lv_obj_set_size(g_hdr, 320, 26);
  lv_obj_set_pos(g_hdr, 0, 0);
  lv_obj_set_style_radius(g_hdr, 0, 0);
  lv_obj_set_style_border_width(g_hdr, 0, 0);
  lv_obj_set_style_pad_hor(g_hdr, 8, 0);
  lv_obj_set_style_pad_ver(g_hdr, 0, 0);
  lv_obj_remove_flag(g_hdr, LV_OBJ_FLAG_SCROLLABLE);
  g_hdrState = lv_label_create(g_hdr);
  lv_obj_set_style_text_font(g_hdrState, &lv_font_montserrat_14, 0);
  lv_obj_set_style_text_color(g_hdrState, lv_color_white(), 0);
  lv_obj_align(g_hdrState, LV_ALIGN_LEFT_MID, 0, 0);
  g_hdrInfo = lv_label_create(g_hdr);
  lv_obj_set_style_text_font(g_hdrInfo, &lv_font_montserrat_12, 0);
  lv_obj_set_style_text_color(g_hdrInfo, lv_color_hex(0xFFD54F), 0);
  lv_obj_align(g_hdrInfo, LV_ALIGN_RIGHT_MID, 0, 0);

  // lista de ajustes: una fila por campo = nombre | slider | valor
  lv_obj_t *list = lv_obj_create(scr);
  lv_obj_set_size(list, 320, 170 - 26);
  lv_obj_set_pos(list, 0, 26);
  lv_obj_set_style_radius(list, 0, 0);
  lv_obj_set_style_border_width(list, 0, 0);
  lv_obj_set_style_bg_opa(list, LV_OPA_TRANSP, 0);
  lv_obj_set_style_pad_all(list, 4, 0);
  lv_obj_set_style_pad_row(list, 2, 0);
  lv_obj_set_flex_flow(list, LV_FLEX_FLOW_COLUMN);
  lv_obj_set_scrollbar_mode(list, LV_SCROLLBAR_MODE_ACTIVE);

  static lv_style_t rowSt, rowSel;
  lv_style_init(&rowSt);
  lv_style_set_radius(&rowSt, 6);
  lv_style_set_border_width(&rowSt, 0);
  lv_style_set_bg_opa(&rowSt, LV_OPA_TRANSP);
  lv_style_set_pad_hor(&rowSt, 8);
  lv_style_set_pad_ver(&rowSt, 0);
  lv_style_set_pad_column(&rowSt, 8);
  lv_style_init(&rowSel);
  lv_style_set_bg_opa(&rowSel, LV_OPA_COVER);
  lv_style_set_bg_color(&rowSel, lv_color_hex(0x1E3A40));

  {  // fila de luz (acción inmediata, no es un ajuste de gestos): Luz [interruptor] [modo]
    lv_obj_t *row = lv_obj_create(list);
    lv_obj_add_style(row, &rowSt, 0);
    lv_obj_set_size(row, LV_PCT(100), 32);
    lv_obj_remove_flag(row, LV_OBJ_FLAG_SCROLLABLE);
    lv_obj_set_flex_flow(row, LV_FLEX_FLOW_ROW);
    lv_obj_set_flex_align(row, LV_FLEX_ALIGN_START, LV_FLEX_ALIGN_CENTER, LV_FLEX_ALIGN_CENTER);
    lv_obj_t *name = lv_label_create(row);
    lv_label_set_text(name, "Luz");
    lv_obj_set_width(name, 104);
    lv_obj_set_style_text_color(name, lv_color_hex(0xE0E0E0), 0);
    g_lightSw = lv_switch_create(row);
    lv_obj_set_size(g_lightSw, 46, 22);
    lv_obj_add_flag(g_lightSw, LV_OBJ_FLAG_SCROLL_ON_FOCUS);
    lv_obj_add_event_cb(g_lightSw, onLightSw, LV_EVENT_VALUE_CHANGED, nullptr);
    lv_group_add_obj(g_grp, g_lightSw);
    lv_obj_t *sp = lv_obj_create(row);  // separador flexible
    lv_obj_remove_style_all(sp);
    lv_obj_set_flex_grow(sp, 1);
    g_lightDd = lv_dropdown_create(row);
    lv_dropdown_set_options(g_lightDd, "Modo 1\nModo 2\nModo 3");
    lv_obj_set_size(g_lightDd, 96, 28);
    lv_obj_set_style_pad_ver(g_lightDd, 4, 0);
    lv_obj_add_flag(g_lightDd, LV_OBJ_FLAG_SCROLL_ON_FOCUS);
    lv_obj_add_event_cb(g_lightDd, onLightDd, LV_EVENT_VALUE_CHANGED, nullptr);
    lv_group_add_obj(g_grp, g_lightDd);
  }

  for (int i = 0; i < settings::kCount; i++) {
    const auto &f = settings::field(i);
    lv_obj_t *row = lv_obj_create(list);
    lv_obj_add_style(row, &rowSt, 0);
    lv_obj_add_style(row, &rowSel, LV_STATE_CHECKED);
    lv_obj_set_size(row, LV_PCT(100), 30);
    lv_obj_remove_flag(row, LV_OBJ_FLAG_SCROLLABLE);
    lv_obj_set_flex_flow(row, LV_FLEX_FLOW_ROW);
    lv_obj_set_flex_align(row, LV_FLEX_ALIGN_START, LV_FLEX_ALIGN_CENTER, LV_FLEX_ALIGN_CENTER);

    lv_obj_t *name = lv_label_create(row);
    lv_label_set_text(name, f.label);
    lv_obj_set_width(name, 104);
    lv_obj_set_style_text_color(name, lv_color_hex(0xE0E0E0), 0);

    lv_obj_t *sl;
    if (f.unit == settings::Unit::Bool) {  // interruptor (p.ej. auto-desarme) + separador para alinear el valor
      sl = lv_switch_create(row);
      lv_obj_set_size(sl, 46, 22);
      lv_obj_t *sp = lv_obj_create(row);
      lv_obj_remove_style_all(sp);
      lv_obj_set_flex_grow(sp, 1);
    } else {
      sl = lv_slider_create(row);
      lv_obj_set_flex_grow(sl, 1);
      lv_obj_set_height(sl, 8);
      lv_slider_set_range(sl, 0, (f.max - f.min) / f.step);
    }
    lv_obj_add_flag(sl, LV_OBJ_FLAG_SCROLL_ON_FOCUS);
    lv_obj_add_event_cb(sl, onSlider, LV_EVENT_VALUE_CHANGED, (void *)(intptr_t)i);
    lv_obj_add_event_cb(sl, onFocus, LV_EVENT_FOCUSED, nullptr);
    lv_obj_add_event_cb(sl, onFocus, LV_EVENT_DEFOCUSED, nullptr);
    if (i == 4 || i == 5) lv_obj_set_style_bg_color(sl, lv_palette_main(LV_PALETTE_ORANGE), LV_PART_INDICATOR);
    lv_group_add_obj(g_grp, sl);
    g_sl[i] = sl;

    lv_obj_t *val = lv_label_create(row);
    lv_obj_set_width(val, 62);
    lv_obj_set_style_text_align(val, LV_TEXT_ALIGN_RIGHT, 0);
    lv_obj_set_style_text_color(val, lv_color_white(), 0);
    g_val[i] = val;
  }
  g_noteShock = lv_label_create(list);  // estado del interruptor de shock (texto lo pone uiUpdateHeader)
  lv_obj_set_style_text_font(g_noteShock, &lv_font_montserrat_12, 0);
  lv_obj_set_style_pad_left(g_noteShock, 8, 0);

  lv_obj_t *btns = lv_obj_create(list);
  lv_obj_add_style(btns, &rowSt, 0);
  lv_obj_set_style_pad_hor(btns, 2, 0);
  lv_obj_set_style_pad_ver(btns, 2, 0);
  lv_obj_set_size(btns, LV_PCT(100), LV_SIZE_CONTENT);
  lv_obj_remove_flag(btns, LV_OBJ_FLAG_SCROLLABLE);
  lv_obj_set_flex_flow(btns, LV_FLEX_FLOW_ROW);
  mkButton(btns, "Guardar", lv_palette_darken(LV_PALETTE_GREEN, 2), onSave);
  mkButton(btns, "Salir", lv_color_hex(0x455A64), onExit);
  mkButton(btns, "Fabrica", lv_color_hex(0x5D4037), onDefaults);
}

static void uiInit() {
  lv_init();
  lv_tick_set_cb([]() -> uint32_t { return millis(); });
  g_disp = lv_display_create(g_lcd.width(), g_lcd.height());
  lv_display_set_flush_cb(g_disp, lvFlush);
  lv_display_set_buffers(g_disp, g_lvBuf, nullptr, sizeof g_lvBuf, LV_DISPLAY_RENDER_MODE_PARTIAL);
  lv_display_set_theme(g_disp, lv_theme_default_init(g_disp, lv_palette_main(LV_PALETTE_TEAL),
                                                     lv_palette_main(LV_PALETTE_AMBER), true, &lv_font_montserrat_14));
  g_grp = lv_group_create();
  lv_group_set_wrap(g_grp, true);
  lv_indev_t *enc = lv_indev_create();
  lv_indev_set_type(enc, LV_INDEV_TYPE_ENCODER);
  lv_indev_set_read_cb(enc, lvEncRead);
  lv_indev_set_group(enc, g_grp);
  uiBuild();
}

static void lcdPower(bool on) {  // llamar con el lock tomado
  if (on) {
    g_lcd.wakeup();
    lv_obj_invalidate(lv_screen_active());
    lv_refr_now(g_disp);  // dibujar ANTES de prender el backlight (no se ve basura vieja)
    digitalWrite(PIN_LCD_BL, HIGH);
  } else {
    digitalWrite(PIN_LCD_BL, LOW);
    g_lcd.sleep();  // SLPIN: el panel queda en µA
  }
}

static void menuOpen() {
  if (!g_lcdOk || g_inMenu || g_m.armed()) return;
  g_edit = g_m.config();
  settings::normalize(g_edit);
  g_inMenu = true;  // desde aquí encoder y botones van a LVGL
  g_menuMask = true;
  g_menuBtn = false;
  g_menuDiff = 0;
  g_menuReq = REQ_NONE;
  g_menuActivity = millis();
  uiLoadValues();
  uiUpdateHeader();
  lv_dropdown_set_selected(g_lightDd, g_lightMode - 1);
  if (g_lightOn) lv_obj_add_state(g_lightSw, LV_STATE_CHECKED);
  else lv_obj_remove_state(g_lightSw, LV_STATE_CHECKED);
  lv_group_focus_obj(g_sl[0]);  // foco inicial en Cadencia, no en la luz (un click perdido no transmite)
  lv_group_set_editing(g_grp, false);
  lv_obj_scroll_to_y(lv_obj_get_parent(lv_obj_get_parent(g_sl[0])), 0, LV_ANIM_OFF);
  lcdLockWait();
  lcdPower(true);
  lcdUnlock();
  rgbLedWrite(14, 0, 0, 8);  // azul tenue = menú
  beep(1200, 60);
  logAdd('M', 1, 0, 0);
}

static void menuClose(bool save) {
  if (!g_inMenu) return;
  bool ok = true;
  if (save) {
    settings::normalize(g_edit);
    NvsStore st(false);
    ok = settings::save(st, g_edit);
    ok = settings::saveLightMode(st, g_lightMode) && ok;
    g_cfg5 = g_edit;
  }
  g_m.reset(g_cfg5, micros());  // SIEMPRE: aplica y limpia KEY/botón "pegados" de la combinación (queda desarmado)
  if (g_pcnt) pcnt_unit_get_count(g_pcnt, &g_lastCount);  // giros del menú no pasan a los gestos
  g_inMenu = false;
  lcdLockWait();
  lcdPower(false);
  lcdUnlock();
  rgbLedWrite(14, 0, 0, 0);
  if (save && ok) { beep(1500, 60); delay(40); beep(2000, 80); }
  else if (save) beep(300, 400);  // falló la NVS
  else beep(700, 80);
  logAdd('M', 0, save, ok);
  Serial.printf("MENU cerrado (%s%s)\n", save ? "guardado" : "sin guardar", save && !ok ? ", ERROR NVS" : "");
}

// Botón con debounce ya resuelto, con el menú abierto. KEY = salir sin guardar.
static void menuButton(int pin, bool down) {
  g_menuActivity = millis();
  if (g_menuMask) {  // esperar a soltar los dos botones de la combinación
    if (!g_btnEnc.pressed && !g_btnKey.pressed) { g_menuMask = false; g_menuDiff = 0; }
    return;
  }
  if (pin == PIN_ENC_BTN) g_menuBtn = down;
  else if (pin == PIN_KEY && down) g_menuReq = REQ_EXIT;
}

static void pollCombo(uint32_t now) {
  static bool on;
  static uint32_t t0;
  if (!g_inMenu && g_lcdOk && g_btnEnc.pressed && g_btnKey.pressed && !g_m.armed()) {
    if (!on) { on = true; t0 = now; }
    else if ((uint32_t)(now - t0) >= COMBO_US) { on = false; menuOpen(); }
  } else {
    on = false;
  }
}

static void pollUi() {
  if (!g_inMenu) return;  // pantalla apagada: LVGL ni corre
  if (g_menuReq != REQ_NONE) {
    MenuReq r = g_menuReq;
    g_menuReq = REQ_NONE;
    menuClose(r == REQ_SAVE);
    return;
  }
  if (millis() - g_menuActivity > MENU_IDLE_MS) { menuClose(false); return; }
  static uint32_t last;
  if (millis() - last < 5) return;
  last = millis();
  uiUpdateHeader();
  if (!lcdLock()) return;  // TX en curso → reintentar en la próxima vuelta
  lv_timer_handler();
  lcdUnlock();
}

static void cmdLcd() {
  Serial.printf("LCD init=%s %dx%d rot=%d menu=%d\n", g_lcdOk ? "OK" : "FAIL", g_lcd.width(), g_lcd.height(),
                g_lcd.getRotation(), g_inMenu);
}

// `menu` abre (como la combinación), `menu save` / `menu exit` cierran. Para probar sin tocar los botones.
static void cmdMenu(int argc, char **argv) {
  if (argc > 1 && !strcmp(argv[1], "save")) g_menuReq = REQ_SAVE;
  else if (argc > 1 && !strcmp(argv[1], "exit")) g_menuReq = REQ_EXIT;
  else if (g_m.armed()) { Serial.println("ERR menú solo desarmado"); return; }
  else { menuOpen(); g_menuMask = false; }  // por serial no hay botones que soltar
  Serial.printf("MENU %s\n", g_inMenu ? "abierto" : "cerrado");
}

// Interruptor de shock (solo por serial, a propósito fuera de la GUI). `shock` = estado, `shock on|off`.
// Encender: tres beeps agudos locales; apagar: uno grave y se vacía la cola de TX (no sale un shock ya encolado).
// Se guarda en NVS en el momento (persiste entre arranques; `nvs clear` lo deja apagado).
static void cmdShock(int argc, char **argv) {
  if (argc > 1 && !strcmp(argv[1], "on")) {
    if (!ALLOW_SHOCK) { Serial.println("ERR compilado con ALLOW_SHOCK=0: el interruptor no puede encenderse"); return; }
    if (!g_shockOn) {
      g_shockOn = true;
      for (int i = 0; i < 3; i++) { beep(2400, 50); delay(50); }
    }
    NvsStore st(false);
    if (!settings::saveShock(st, true)) Serial.println("ERR NVS: el estado no quedó guardado");
  } else if (argc > 1 && !strcmp(argv[1], "off")) {
    if (g_shockOn) {
      g_shockOn = false;
      xQueueReset(g_txQueue);
      beep(500, 150);
    }
    NvsStore st(false);
    if (!settings::saveShock(st, false)) Serial.println("ERR NVS: el estado no quedó guardado");
  } else if (argc > 1) {
    Serial.println("ERR uso: shock [on|off]");
    return;
  }
  logAdd('S', g_shockOn, 0, 0);
  Serial.printf("SHOCK %s (ALLOW_SHOCK=%d; guardado en NVS)\n", shockEnabled() ? "ON" : "off", ALLOW_SHOCK);
}

// `nvs` muestra lo guardado; `nvs clear` borra el namespace y vuelve a defaults (desarmado).
static void cmdNvs(int argc, char **argv) {
  if (argc > 1 && !strcmp(argv[1], "clear")) {
    { Preferences p; if (p.begin("t60", false)) { p.clear(); p.end(); } }
    g_cfg5 = gesture::defaults();
    g_lightMode = 1;
    g_shockOn = false;
    g_m.reset(g_cfg5, micros());
    Serial.println("NVS borrada, defaults aplicados (desarmado)");
    return;
  }
  NvsStore st(true);
  Serial.printf("NVS t60 %s (al arrancar se leyeron %d campos):", st.ok ? "OK" : "vacía", g_nvsFound);
  int32_t v;
  if (st.get("schema", v)) Serial.printf(" schema=%ld", (long)v);
  if (st.get("lmode", v)) Serial.printf(" lmode=%ld", (long)v);
  if (st.get("shock", v)) Serial.printf(" shock=%ld", (long)v);
  for (int i = 0; i < settings::kCount; i++)
    if (st.get(settings::field(i).key, v)) Serial.printf(" %s=%ld", settings::field(i).key, (long)v);
  Serial.println();
}

static void cmdLog() {
  uint32_t n = g_logN, from = n > 64 ? n - 64 : 0;
  Serial.printf("LOG %u eventos (últimos %u). I=entrada(ev) D=decisión(0pulso/1tono+/2tono-,modo,pot) F=trama(modo,pot,seq)\n",
                n, n - from);
  for (uint32_t k = from; k < n; k++) {
    LogE e = g_log[k & 255];
    Serial.printf("  %10u %c %3u %3u %3u\n", e.t, e.kind, e.a, e.b, e.c);
  }
}
static void cmdGstat() {
  Serial.printf("GSTAT armado=%d pulsación=%d nivel_rampa=%u | gap=%u ms idle=%u ms grace=%u ms key=%u ms maxramp=%u ms "
                "BAJO=%u ALTO=%u TOPE=%u paso=%u wiggle=%u zona=%u autodesarme=%s/%lus ALLOW_SHOCK=%d shock=%s "
                "luz_modo=%ld\n",
                g_m.armed(), g_m.pressActive(), g_m.rampLevel(), g_m.config().minStimGapUs / 1000,
                g_m.config().rampIdleUs / 1000, g_m.config().armGraceUs / 1000, g_m.config().keyHoldUs / 1000,
                g_m.config().maxRampUs / 1000, g_m.config().lowPower, g_m.config().highPower, g_m.config().cap,
                g_m.config().step, g_m.config().wiggleRevs, g_m.config().safeClicks,
                g_m.config().autoDisarm ? "on" : "OFF", (unsigned long)(g_m.config().autoDisarmUs / 1000000),
                ALLOW_SHOCK, shockEnabled() ? "ON" : "off",
                (long)g_lightMode);
}

// Ajusta un parámetro en vivo (re-inicia la máquina → queda desarmada; re-armar con KEY). Para afinar la cadencia
// y niveles sin reflashear. `set <gap|idle|key|maxramp|bajo|alto|tope|paso> <valor_ms_o_nivel>`.
static void cmdSet(int argc, char **argv) {
  if (argc < 3) {
    Serial.println("ERR uso: set <gap|idle|key|maxramp|bajo|alto|tope|paso|wiggle|wgap|keytono|keyvibr> <valor>");
    return;
  }
  int v = atoi(argv[2]);
  const char *f = argv[1];
  if (!strcmp(f, "gap")) g_cfg5.minStimGapUs = (uint32_t)v * 1000;
  else if (!strcmp(f, "idle")) g_cfg5.rampIdleUs = (uint32_t)v * 1000;
  else if (!strcmp(f, "key")) g_cfg5.keyHoldUs = (uint32_t)v * 1000;
  else if (!strcmp(f, "maxramp")) g_cfg5.maxRampUs = (uint32_t)v * 1000;
  else if (!strcmp(f, "bajo")) g_cfg5.lowPower = v;
  else if (!strcmp(f, "alto")) g_cfg5.highPower = v;
  else if (!strcmp(f, "tope")) g_cfg5.cap = v;
  else if (!strcmp(f, "paso")) g_cfg5.step = v;
  else if (!strcmp(f, "wiggle")) g_cfg5.wiggleRevs = v;      // reversas para armar/desarmar
  else if (!strcmp(f, "wgap")) g_cfg5.wiggleGapUs = (uint32_t)v * 1000;  // ms máx entre reversas
  else if (!strcmp(f, "grace")) g_cfg5.armGraceUs = (uint32_t)v * 1000;  // ms de gracia tras armar
  else if (!strcmp(f, "keytono")) g_cfg5.keyTonoPower = v;
  else if (!strcmp(f, "keyvibr")) g_cfg5.keyVibrPower = v;
  else { Serial.println("ERR campo: gap idle key maxramp bajo alto tope paso wiggle wgap keytono keyvibr"); return; }
  g_m.reset(g_cfg5, micros());  // re-aplica (desarma)
  Serial.print("SET ok (desarmado, re-armá con KEY). ");
  cmdGstat();
}

// ---------------------------------------------------------------- setup/loop
void setup() {
  pinMode(PIN_GDO0, INPUT);  // hasta que GDO0 quede en alta impedancia en el CC1101
  pinMode(PIN_GDO2, INPUT);
  for (int p : {PIN_LCD_CS, PIN_SD_CS, PIN_NRF_CS, PIN_CSN}) { pinMode(p, OUTPUT); digitalWrite(p, HIGH); }
  pinMode(PIN_SW1, OUTPUT); digitalWrite(PIN_SW1, HIGH);
  pinMode(PIN_SW0, OUTPUT); digitalWrite(PIN_SW0, HIGH);
  pinMode(PIN_PWR_EN, OUTPUT); digitalWrite(PIN_PWR_EN, HIGH);
  for (int p : {PIN_ENC_A, PIN_ENC_B, PIN_ENC_BTN, PIN_KEY}) pinMode(p, INPUT_PULLUP);  // punto 2: medición
  Serial.begin(115200);
  // Nunca bloquear por el USB: con el cable puesto y nadie leyendo el puerto, HWCDC espera hasta 20 × 100 ms por
  // escritura (HWCDC.cpp, "host backpressure"). El `TX final` de cada pulso hacía tardar ~2.27 s cada envío de la
  // tarea de TX → la rampa se encolaba y perdía pulsos (medido en `log` 2026-10-20). Sin lector, se descarta.
  Serial.setTxTimeoutMs(0);
  delay(100);

  g_spiLock = xSemaphoreCreateMutex();
  g_rfMutex = xSemaphoreCreateMutex();
  // LCD ANTES que el CC1101: su init hace SPI.end()/SPI.begin() y resetea el periférico FSPI (ver lcd.h).
  pinMode(PIN_LCD_BL, OUTPUT);
  digitalWrite(PIN_LCD_BL, LOW);
  g_lcdOk = g_lcd.init();
  if (g_lcdOk) {
    g_lcd.setRotation(3);
    g_lcd.fillScreen(TFT_BLACK);
    g_lcd.sleep();  // la pantalla vive apagada; la enciende la combinación KEY + botón (fase 3)
  }
  spi.begin(PIN_SCK, PIN_MISO, PIN_MOSI, -1);  // no-op si el LCD ya lo inició (mismos pines)
  cc1101Reset();
  cc1101Config();  // IOCFG0=0x2E antes de que el RMT tome GPIO3 como salida
  rmtInit();

  xTaskCreatePinnedToCore(wdTask, "t60wd", 3072, nullptr, configMAX_PRIORITIES - 2, &g_wdTask, 0);

  g_edges = (Edge *)heap_caps_malloc(MAX_EDGES * sizeof(Edge), MALLOC_CAP_INTERNAL);
  g_enc = (EncT *)heap_caps_malloc(MAX_ENC * sizeof(EncT), MALLOC_CAP_INTERNAL);
  g_seq = esp_random() & 0xFE;

  // --- fase 5: control por gestos ---
  g_txSeq = esp_random() & 0xFE;
  g_txQueue = xQueueCreate(16, sizeof(TxCmd));
  {  // ajustes guardados desde el menú (NVS "t60"), recortados a rango; sin NVS → defaults
    NvsStore st(true);
    g_cfg5 = settings::load(st, gesture::defaults(), &g_nvsFound);
    g_lightMode = settings::loadLightMode(st);
    g_shockOn = ALLOW_SHOCK && settings::loadShock(st);
  }
  g_m.reset(g_cfg5, micros());  // arranca DESARMADO
  encoderInit();
  audioInit();  // parlante para beeps locales (confirmación de armado); nunca estimula el collar
  xTaskCreatePinnedToCore(txTask, "t60tx", 4096, nullptr, 3, nullptr, 0);
  rgbLedWrite(14, 0, 0, 0);  // LED apagado = desarmado
  if (g_lcdOk) uiInit();     // LVGL listo pero sin correr: la pantalla queda apagada hasta la combinación
}

void loop() {
  static char line[128];
  static size_t len = 0;
  while (Serial.available()) {
    char ch = Serial.read();
    if (ch == '\r' || ch == '\n') {
      line[len] = 0;
      if (len) handle(line);
      len = 0;
    } else if (len < sizeof(line) - 1) {
      line[len++] = ch;
    }
  }
  pollGestures();  // encoder + botones + máquina de gestos + feedback (no bloquea; la TX va en su tarea)
  pollUi();        // menú LVGL: solo con la pantalla encendida y el RF libre
}
