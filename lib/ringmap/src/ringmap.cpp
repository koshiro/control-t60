#include "ringmap.h"

#include <string.h>

namespace ringmap {

static uint32_t fnv(uint32_t h, const uint8_t *d, size_t n) {
  for (size_t i = 0; i < n; i++) h = (h ^ d[i]) * 16777619u;
  return h;
}

void clear(Table &t) {
  memset(&t, 0, sizeof t);
  t.version = kVersion;
}

bool valid(const Table &t) {
  if (t.version != kVersion || t.count > kMaxSigs) return false;
  for (int i = 0; i < t.count; i++) {
    const Sig &s = t.sigs[i];
    if (s.button >= kButtons || s.clicks < 1 || s.clicks > 2 || s.n < 1 || s.n > kMaxBurst) return false;
  }
  return true;
}

bool isRelease(const uint8_t *data, size_t len) {
  for (size_t i = 0; i < len; i++)
    if (data[i]) return false;
  return true;
}

uint32_t reportHash(uint16_t src, const uint8_t *data, size_t len) {
  uint8_t head[3] = {(uint8_t)src, (uint8_t)(src >> 8), (uint8_t)len};
  return fnv(fnv(2166136261u, head, sizeof head), data, len);
}

uint32_t seqHash(const uint32_t *h, int n) {
  return fnv(2166136261u ^ (uint32_t)n, (const uint8_t *)h, (size_t)n * sizeof(uint32_t));
}

int decode(const Table &t, const uint32_t *h, int n, uint8_t *btn, int maxOut) {
  if (!valid(t) || n <= 0) return 0;
  int clicks = 0;
  for (int pos = 0; pos < n;) {
    const Sig *best = nullptr;
    for (int i = 0; i < t.count; i++) {
      const Sig &s = t.sigs[i];
      if (pos + s.n > n || (best && s.n <= best->n)) continue;
      if (seqHash(h + pos, s.n) == s.hash) best = &s;
    }
    if (!best) return 0;  // trozo desconocido → la ráfaga entera no cuenta
    for (int k = 0; k < best->clicks; k++) {
      if (clicks < maxOut) btn[clicks] = best->button;
      clicks++;
    }
    pos += best->n;
  }
  return clicks;
}

bool settled(const Table &t, const uint32_t *h, int n) {
  if (!valid(t) || n <= 0) return false;
  for (int pos = 0; pos < n;) {
    const Sig *best = nullptr;
    for (int i = 0; i < t.count; i++) {
      const Sig &s = t.sigs[i];
      if (pos + s.n > n || (best && s.n <= best->n)) continue;
      if (seqHash(h + pos, s.n) == s.hash) best = &s;
    }
    if (!best) return false;
    if (pos + best->n == n) {  // último trozo: ¿podría crecer hasta una firma más larga?
      for (int i = 0; i < t.count; i++)
        if (t.sigs[i].n > best->n && t.sigs[i].first == h[pos]) return false;
      return true;
    }
    pos += best->n;
  }
  return false;
}

Learn learn(Table &t, const uint32_t *h, int n, uint8_t button, uint8_t clicks) {
  if (!valid(t)) clear(t);
  if (n <= 0) return Learn::Empty;
  if (n > kMaxBurst || button >= kButtons || clicks < 1 || clicks > 2) return Learn::Full;
  uint8_t got[8];
  int c = decode(t, h, n, got, 8);
  if (c > 0) {
    bool same = c == clicks;
    for (int i = 0; same && i < c && i < 8; i++) same = got[i] == button;
    return same ? Learn::Known : Learn::Conflict;
  }
  if (t.count >= kMaxSigs) return Learn::Full;
  Sig &s = t.sigs[t.count++];
  s.hash = seqHash(h, n);
  s.n = (uint8_t)n;
  s.button = button;
  s.clicks = clicks;
  s.pad = 0;
  s.first = h[0];
  return Learn::Added;
}

void Burst::add(uint32_t h, uint32_t nowUs) {
  if (n_ < kMaxBurst) h_[n_++] = h;
  else overflow_ = true;
  last_ = nowUs;
}

}  // namespace ringmap
