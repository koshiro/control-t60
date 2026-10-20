#include "t60frame.h"

namespace t60 {

uint8_t checksum(const Frame &f) {
  return (uint8_t)(f.idHi + f.idLo + f.seq + f.mode + f.power);
}

void payload(const Frame &f, uint8_t out[6]) {
  out[0] = f.idHi;
  out[1] = f.idLo;
  out[2] = f.seq;
  out[3] = f.mode;
  out[4] = f.power;
  out[5] = checksum(f) ^ f.chkXor;
}

void frameBits(const Frame &f, uint8_t *out) {
  int n = 0;
  for (int i = 0; i < kPreambleBits; i++) out[n++] = (i & 1) ? 0 : 1;
  out[n++] = 1;
  out[n++] = 1;
  uint8_t bytes[7] = {0x30};
  payload(f, bytes + 1);
  for (uint8_t b : bytes)
    for (int k = 7; k >= 0; k--) out[n++] = (b >> k) & 1;
}

// Agrega `bits` a la lista de corridas, fusionando con la última si tiene el mismo nivel.
static bool push(Run *out, int max, int &n, uint8_t level, uint32_t tenths) {
  if (n > 0 && out[n - 1].level == level) {
    out[n - 1].tenthsUs += tenths;
    return true;
  }
  if (n >= max) return false;
  out[n++] = {level, tenths};
  return true;
}

static bool appendFrame(const Frame &f, Run *out, int max, int &n) {
  uint8_t bits[kFrameBits];
  frameBits(f, bits);
  for (uint8_t b : bits)
    if (!push(out, max, n, b, kBitTenthsUs)) return false;
  return push(out, max, n, 0, kGapTenthsUs);
}

int pressRuns(const Frame &f, int frames, bool release, Run *out, int max) {
  int n = 0;
  for (int i = 0; i < frames; i++)
    if (!appendFrame(f, out, max, n)) return -1;
  if (release) {
    Frame r = f;
    r.seq = f.seq + 1;
    r.mode = kReleaseMode;
    r.power = kReleasePower;
    if (!appendFrame(r, out, max, n)) return -1;
  }
  return n;
}

int rampRuns(uint8_t seq, uint8_t mode, const uint8_t *powers, int nPowers, int perFrame, bool release,
             Run *out, int max) {
  int n = 0;
  Frame f;
  f.seq = seq;
  f.mode = mode;
  for (int i = 0; i < nPowers; i++) {
    f.power = powers[i];
    for (int k = 0; k < perFrame; k++)
      if (!appendFrame(f, out, max, n)) return -1;
  }
  if (release) {
    Frame r;
    r.seq = (uint8_t)(seq + 1);
    r.mode = kReleaseMode;
    r.power = kReleasePower;
    if (!appendFrame(r, out, max, n)) return -1;
  }
  return n;
}

}  // namespace t60
