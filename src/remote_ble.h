// Servidor BLE del botón remoto (un solo botón de emergencia). El remoto NO decide nada: manda flancos del botón
// (1 = apretado, 0 = suelto) y la máquina de gestos del T-Embed (gesture::Machine, In::RemoteDown/RemoteUp) los
// interpreta con SU reloj: 1 tap = BAJO, 2 = ALTO, 3+ = rampa. Armar/desarmar sigue siendo local (wiggle de la rueda).
// Seguridad: el enlace va cifrado y autenticado (LE Secure Connections + passkey DINÁMICA, bonding). Cada emparejamiento
// genera un PIN aleatorio de 6 dígitos que el firmware muestra en la pantalla del T-Embed (takePairEvent); no hay PIN
// guardado en ningún lado. Las características exigen cifrado, así que sin emparejar no se puede ni escribir ni leer.
// Las callbacks corren en la tarea del host NimBLE: SOLO encolan; el loop() alimenta la máquina (no es thread-safe).
#pragma once
#include <NimBLEDevice.h>
#include <freertos/FreeRTOS.h>
#include <freertos/queue.h>

namespace remote {

static constexpr const char *kService = "6e7a0001-b5a3-f393-e0a9-e50e24dcca9e";
static constexpr const char *kCmdChar = "6e7a0002-b5a3-f393-e0a9-e50e24dcca9e";     // write (sin respuesta): 1 down / 0 up
static constexpr const char *kStatusChar = "6e7a0003-b5a3-f393-e0a9-e50e24dcca9e";  // read + notify: [armado, shock, rampa, nº pulsos]

enum Ev : uint8_t { Up = 0, Down = 1, Lost = 0xFE };  // Lost: el enlace cayó (descartar taps pendientes)

static QueueHandle_t g_q;
static NimBLEServer *g_srv;
static NimBLECharacteristic *g_status;
static volatile bool g_conn;
static volatile uint32_t g_pin;     // PIN del emparejamiento en curso
static volatile uint8_t g_pairEv;   // 0 nada · 1 mostrar PIN · 2 emparejamiento terminado (ocultar)

static void push(uint8_t ev) {
  if (g_q) xQueueSend(g_q, &ev, 0);
}

class SrvCb : public NimBLEServerCallbacks {
  void onConnect(NimBLEServer *s, NimBLEConnInfo &ci) override {
    g_conn = true;
    s->updateConnParams(ci.getConnHandle(), 12, 24, 0, 400);  // 15–30 ms, sin latencia, timeout 4 s: respuesta ágil
  }
  void onDisconnect(NimBLEServer *, NimBLEConnInfo &, int) override {
    g_conn = false;
    g_pairEv = 2;  // si había un PIN en pantalla, ocultarlo
    push(Lost);
    NimBLEDevice::startAdvertising();
  }
  uint32_t onPassKeyDisplay() override {  // el host pide el PIN a mostrar: uno nuevo por emparejamiento
    g_pin = esp_random() % 1000000;
    g_pairEv = 1;
    return g_pin;
  }
  void onAuthenticationComplete(NimBLEConnInfo &ci) override {
    g_pairEv = 2;
    if (!ci.isEncrypted() || !ci.isAuthenticated()) NimBLEDevice::getServer()->disconnect(ci.getConnHandle());
  }
};

class CmdCb : public NimBLECharacteristicCallbacks {
  void onWrite(NimBLECharacteristic *c, NimBLEConnInfo &) override {
    NimBLEAttValue v = c->getValue();
    if (v.size() < 1) return;
    uint8_t b = v.data()[0];
    if (b == 0 || b == 1) push(b);  // cualquier otro byte se ignora
  }
};

static SrvCb g_srvCb;
static CmdCb g_cmdCb;

inline void begin(const char *name) {
  if (!g_q) g_q = xQueueCreate(16, sizeof(uint8_t));
  g_conn = false;
  g_pairEv = 0;
  NimBLEDevice::init(name);
  NimBLEDevice::setSecurityAuth(true, true, true);  // bonding + MITM + Secure Connections
  NimBLEDevice::setSecurityIOCap(BLE_HS_IO_DISPLAY_ONLY);  // sin passkey estática: el PIN sale de onPassKeyDisplay
  g_srv = NimBLEDevice::createServer();
  g_srv->setCallbacks(&g_srvCb);
  NimBLEService *svc = g_srv->createService(kService);
  NimBLECharacteristic *cmd = svc->createCharacteristic(
      kCmdChar, NIMBLE_PROPERTY::WRITE_NR | NIMBLE_PROPERTY::WRITE | NIMBLE_PROPERTY::WRITE_ENC | NIMBLE_PROPERTY::WRITE_AUTHEN);
  cmd->setCallbacks(&g_cmdCb);
  g_status = svc->createCharacteristic(
      kStatusChar, NIMBLE_PROPERTY::READ | NIMBLE_PROPERTY::NOTIFY | NIMBLE_PROPERTY::READ_ENC | NIMBLE_PROPERTY::READ_AUTHEN);
  uint8_t zero[4] = {0, 0, 0, 0};
  g_status->setValue(zero, sizeof zero);
  svc->start();
  NimBLEAdvertising *adv = NimBLEDevice::getAdvertising();
  adv->addServiceUUID(kService);
  adv->setName(name);
  adv->start();
}

// Apagar el BLE entero (antes de un light sleep: el controlador no sobrevive dormido). Los emparejados quedan en NVS.
inline void end() {
  NimBLEDevice::deinit(true);
  g_srv = nullptr;
  g_status = nullptr;
  g_conn = false;
  g_pairEv = 0;
}

inline bool connected() { return g_conn; }
// Evento de emparejamiento para la pantalla: devuelve 1 (mostrar `pin`), 2 (ocultar) o 0; lo consume.
inline uint8_t takePairEvent(uint32_t &pin) {
  uint8_t e = g_pairEv;
  if (e) { pin = g_pin; g_pairEv = 0; }
  return e;
}
inline bool pop(uint8_t &ev) { return g_q && xQueueReceive(g_q, &ev, 0) == pdTRUE; }

// Estado para la app (se notifica solo si cambió).
inline void publish(bool armed, bool shock, uint8_t ramp, uint8_t nPulses) {
  static uint8_t last[4] = {0xFF, 0xFF, 0xFF, 0xFF};
  uint8_t cur[4] = {(uint8_t)armed, (uint8_t)shock, ramp, nPulses};
  if (!g_status || !memcmp(cur, last, sizeof cur)) return;
  memcpy(last, cur, sizeof cur);
  g_status->setValue(cur, sizeof cur);
  if (g_conn) g_status->notify();
}

inline void clearBonds() { NimBLEDevice::deleteAllBonds(); }
inline int bondCount() { return NimBLEDevice::getNumBonds(); }

}  // namespace remote
