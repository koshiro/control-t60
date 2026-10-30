// Servidor BLE del botón remoto (un solo botón de emergencia). El remoto NO decide nada: manda flancos del botón
// (1 = apretado, 0 = suelto) y la máquina de gestos del T-Embed (gesture::Machine, In::RemoteDown/RemoteUp) los
// interpreta con SU reloj: 1 tap = BAJO, 2 = ALTO, 3+ = rampa. Armar/desarmar sigue siendo local (wiggle de la rueda).
// Seguridad: el enlace va cifrado y autenticado (LE Secure Connections + passkey DINÁMICA, bonding). Cada emparejamiento
// genera un PIN aleatorio de 6 dígitos que el firmware muestra en la pantalla del T-Embed (takePairEvent); no hay PIN
// guardado en ningún lado. Las características exigen cifrado, así que sin emparejar no se puede ni escribir ni leer.
// BLE nativo del core Arduino-ESP32 (NimBLE integrado; NO usar NimBLE-Arduino: duplica el host y corrompe la RAM).
// Las callbacks corren en la tarea del host NimBLE: SOLO encolan; el loop() alimenta la máquina (no es thread-safe).
#pragma once
#include <BLEDevice.h>
#include <BLEServer.h>
#include <BLEUtils.h>
#include <esp_heap_caps.h>
#include <freertos/FreeRTOS.h>
#include <freertos/queue.h>
#include <host/ble_store.h>

namespace remote {

static constexpr const char *kService = "6e7a0001-b5a3-f393-e0a9-e50e24dcca9e";
static constexpr const char *kCmdChar = "6e7a0002-b5a3-f393-e0a9-e50e24dcca9e";     // write (sin respuesta): 1 down / 0 up
static constexpr const char *kStatusChar = "6e7a0003-b5a3-f393-e0a9-e50e24dcca9e";  // read + notify: [armado, shock, rampa, nº pulsos]

enum Ev : uint8_t { Up = 0, Down = 1, Lost = 0xFE };  // Lost: el enlace cayó (descartar taps pendientes)

static QueueHandle_t g_q;
static BLEServer *g_srv;
static BLECharacteristic *g_status;
static volatile bool g_conn;
static volatile uint32_t g_pin;     // PIN del emparejamiento en curso
static volatile uint8_t g_pairEv;   // 0 nada · 1 mostrar PIN · 2 emparejamiento terminado (ocultar)
static bool g_initOk;               // diagnóstico (`ble`)
static uint32_t g_heapBefore, g_heapAfter;

static void push(uint8_t ev) {
  if (g_q) xQueueSend(g_q, &ev, 0);
}

class SrvCb : public BLEServerCallbacks {
  void onConnect(BLEServer *s, ble_gap_conn_desc *d) override {
    g_conn = true;
    s->updateConnParams(d->conn_handle, 12, 24, 0, 400);  // 15–30 ms, sin latencia, timeout 4 s: respuesta ágil
  }
  void onDisconnect(BLEServer *, ble_gap_conn_desc *) override {
    g_conn = false;
    g_pairEv = 2;  // si había un PIN en pantalla, ocultarlo
    push(Lost);
    BLEDevice::startAdvertising();
  }
};

class SecCb : public BLESecurityCallbacks {
  void onPassKeyNotify(uint32_t k) override {  // PIN aleatorio nuevo por emparejamiento: mostrarlo en la pantalla
    g_pin = k;
    g_pairEv = 1;
  }
  bool onSecurityRequest() override { return true; }
  void onAuthenticationComplete(ble_gap_conn_desc *d, int status) override {
    g_pairEv = 2;
    if (status != 0 || !d->sec_state.encrypted || !d->sec_state.authenticated) g_srv->disconnect(d->conn_handle);
  }
};

class CmdCb : public BLECharacteristicCallbacks {
  void onWrite(BLECharacteristic *c, ble_gap_conn_desc *) override {
    if (c->getLength() < 1) return;
    uint8_t b = c->getData()[0];
    if (b == 0 || b == 1) push(b);  // cualquier otro byte se ignora
  }
};

static SrvCb g_srvCb;
static SecCb g_secCb;
static CmdCb g_cmdCb;

inline void begin(const char *name) {
  if (!g_q) g_q = xQueueCreate(16, sizeof(uint8_t));
  g_conn = false;
  g_pairEv = 0;
  g_heapBefore = heap_caps_get_largest_free_block(MALLOC_CAP_INTERNAL);
  BLEDevice::init(name);
  g_initOk = true;
  BLESecurity::setAuthenticationMode(true, true, true);  // bonding + MITM + Secure Connections
  BLESecurity::setCapability(ESP_IO_CAP_OUT);           // solo pantalla: el PIN se muestra aquí y se teclea en el Mac
  BLESecurity::setPassKey(false);                       // aleatorio...
  BLESecurity::regenPassKeyOnConnect(true);             // ...y nuevo en cada conexión
  BLEDevice::setSecurityCallbacks(&g_secCb);
  g_srv = BLEDevice::createServer();
  g_srv->setCallbacks(&g_srvCb);
  BLEService *svc = g_srv->createService(kService);
  BLECharacteristic *cmd = svc->createCharacteristic(
      kCmdChar, BLECharacteristic::PROPERTY_WRITE_NR | BLECharacteristic::PROPERTY_WRITE | BLECharacteristic::PROPERTY_WRITE_ENC |
                    BLECharacteristic::PROPERTY_WRITE_AUTHEN);
  cmd->setCallbacks(&g_cmdCb);
  g_status = svc->createCharacteristic(
      kStatusChar, BLECharacteristic::PROPERTY_READ | BLECharacteristic::PROPERTY_NOTIFY | BLECharacteristic::PROPERTY_READ_ENC |
                       BLECharacteristic::PROPERTY_READ_AUTHEN);
  uint8_t zero[4] = {0, 0, 0, 0};
  g_status->setValue(zero, sizeof zero);
  svc->start();
  BLEAdvertising *adv = BLEDevice::getAdvertising();
  adv->addServiceUUID(kService);
  adv->setScanResponse(true);
  BLEDevice::startAdvertising();
  g_heapAfter = heap_caps_get_largest_free_block(MALLOC_CAP_INTERNAL);
}

// Apagar el BLE entero (antes de un light sleep: el controlador no sobrevive dormido). Los emparejados quedan en NVS.
inline void end() {
  BLEDevice::deinit(false);
  g_srv = nullptr;
  g_status = nullptr;
  g_conn = false;
  g_pairEv = 0;
}

inline bool initOk() { return g_initOk; }
inline uint32_t heapBefore() { return g_heapBefore; }
inline uint32_t heapAfter() { return g_heapAfter; }
inline bool started() { return g_srv != nullptr; }
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

inline void clearBonds() { if (g_srv) ble_store_clear(); }
inline int bondCount() {
  int n = 0;
  return g_srv && ble_store_util_count(BLE_STORE_OBJ_TYPE_OUR_SEC, &n) == 0 ? n : -1;  // -1: BLE apagado
}

}  // namespace remote
