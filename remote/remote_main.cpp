// Control remoto BLE de UN solo botón (ESP32-C3 Super Mini + CR2032) para el T-Embed con firmware control-t60.
// Cliente BLE que duerme en deep sleep y despierta al pulsar el botón (GPIO3 a GND). Manda solo flancos del botón
// (1 = apretado, 0 = suelto) a la característica cifrada del T-Embed; la máquina de gestos del T-Embed decide
// (1 toque = bajo, 2 = alto, 3+ = rampa). Seguridad y emparejamiento: patrón oficial `Client_secure_static_passkey` del
// core Arduino-ESP32 (passkey compartido BLE_REMOTE_PIN de `.env`, ESP_IO_CAP_IN); el T-Embed debe tener abierta la
// ventana de emparejamiento del remoto (KEY 2 s desarmado, o `ble pair remoto`) la primera vez.
#include <Arduino.h>
#include <BLEDevice.h>
#include <BLESecurity.h>
#include <Preferences.h>
#include <driver/gpio.h>
#include <esp_sleep.h>

#ifndef BLE_REMOTE_PIN
#error "BLE_REMOTE_PIN lo define tools/build_env_remote.py desde .env"
#endif
#ifndef FW_VERSION
#define FW_VERSION "sin-version"
#endif

static constexpr gpio_num_t PIN_BTN = GPIO_NUM_3;  // botón entre GPIO3 y GND (pull-up interno)
static constexpr uint32_t DEBOUNCE_MS = 15;
static constexpr uint32_t AWAKE_MS = 6000;         // despierto tras el último evento (ventana para 2º/3er toque y rampa)
static constexpr uint32_t CONNECT_BUDGET_MS = 12000;  // si no hay enlace en este tiempo, volver a dormir (no gastar pila)

static const BLEUUID kService("6e7a0001-b5a3-f393-e0a9-e50e24dcca9e");
static const BLEUUID kCmdChar("6e7a0002-b5a3-f393-e0a9-e50e24dcca9e");
static const BLEUUID kStatusChar("6e7a0003-b5a3-f393-e0a9-e50e24dcca9e");  // lectura cifrada: sirve para disparar la seguridad

static BLEClient *g_client;
static BLERemoteCharacteristic *g_cmd;
static volatile bool g_connected;
static Preferences g_prefs;

// Eventos del botón con su hora (ms desde el arranque). Los registra una TAREA aparte para no perder ningún flanco mientras
// setup() está bloqueado conectando (1-3 s): antes se perdía el UP si se soltaba antes de conectar.
struct Ev { uint32_t ts; uint8_t down; };
static QueueHandle_t g_evq;
static uint32_t g_t0;

class ClientCb : public BLEClientCallbacks {
  void onConnect(BLEClient *) override { Serial.println("enlace: conectado"); }
  void onDisconnect(BLEClient *) override {
    g_connected = false;
    Serial.println("enlace: desconectado");
  }
};
class SecCb : public BLESecurityCallbacks {
  void onAuthenticationComplete(ble_gap_conn_desc *, int status) override {
    Serial.printf("seguridad: %s (status=%d)\n", status == 0 ? "OK" : "FALLÓ", status);
  }
};

class ScanCb : public BLEAdvertisedDeviceCallbacks {
 public:
  BLEAdvertisedDevice *found = nullptr;
  void onResult(BLEAdvertisedDevice d) override {
    if (d.haveServiceUUID() && d.isAdvertisingService(kService) && !found) {
      found = new BLEAdvertisedDevice(d);
      BLEDevice::getScan()->stop();
    }
  }
};

static bool btnDown() { return digitalRead(PIN_BTN) == LOW; }

static void pushEv(uint32_t ts, uint8_t down) {
  Ev e{ts, down};
  xQueueSend(g_evq, &e, 0);
}

static void btnTask(void *) {
  bool last = btnDown();  // el pulso que nos despertó ya se encoló en setup() (ts=0); aquí solo los flancos nuevos
  uint32_t tEdge = 0;
  for (;;) {
    bool now = btnDown();
    if (now != last && millis() - tEdge >= DEBOUNCE_MS) {
      tEdge = millis();
      last = now;
      pushEv(millis() - g_t0, now ? 1 : 0);
    }
    vTaskDelay(1);
  }
}

// Conecta al T-Embed: primero por la dirección guardada (rápido), si no escaneando por el servicio. Devuelve true con la
// característica de comandos lista y el enlace cifrado.
static bool connectToTembed() {
  if (!g_client) {
    g_client = BLEDevice::createClient();
    g_client->setClientCallbacks(new ClientCb());
  }
  bool ok = false;
  String saved = g_prefs.getString("addr", "");
  uint8_t savedType = g_prefs.getUChar("atype", 0);
  if (saved.length()) {
    Serial.printf("conectando a la dirección guardada %s\n", saved.c_str());
    ok = g_client->connect(BLEAddress(saved, savedType), savedType, 4000);
  }
  if (!ok) {
    Serial.println("escaneando el servicio del T-Embed");
    ScanCb *cb = new ScanCb();
    BLEScan *scan = BLEDevice::getScan();
    scan->setAdvertisedDeviceCallbacks(cb);
    scan->setInterval(100);
    scan->setWindow(99);
    scan->setActiveScan(true);
    scan->start(5, false);
    scan->clearResults();
    if (!cb->found) { Serial.println("no encontré el T-Embed"); return false; }
    ok = g_client->connect(cb->found);
    if (ok) {
      g_prefs.putString("addr", cb->found->getAddress().toString().c_str());
      g_prefs.putUChar("atype", cb->found->getAddressType());
    }
  }
  if (!ok) { Serial.println("no se pudo conectar"); return false; }
  g_connected = true;
  BLERemoteService *svc = g_client->getService(kService);
  g_cmd = svc ? svc->getCharacteristic(kCmdChar) : nullptr;
  if (!g_cmd) { Serial.println("servicio o característica no encontrados"); g_client->disconnect(); return false; }
  // Como el ejemplo oficial Client_secure_static_passkey: en NimBLE la seguridad se negocia "a demanda" al leer una
  // característica protegida. (BLEClient::secureConnection() espera el cifrado con BLE_NPL_TIME_FOREVER y se colgaba.) Con
  // bond reutiliza la clave; sin bond empareja con el passkey compartido (el T-Embed debe tener la ventana del remoto
  // abierta). Una escritura sin respuesta NO sirve para esto: si el permiso falta, el servidor la descarta en silencio.
  BLERemoteCharacteristic *st = svc->getCharacteristic(kStatusChar);
  if (!st || !st->canRead()) { Serial.println("característica de estado no encontrada"); g_client->disconnect(); return false; }
  String v = st->readValue();
  if (v.length() < 4 || !g_client->isConnected()) {
    Serial.println("el cifrado falló (¿ventana de emparejamiento cerrada o passkey distinto?)");
    g_client->disconnect();
    return false;
  }
  Serial.printf("T-Embed: armado=%d shock=%d rampa=%d\n", v[0], v[1], v[2]);
  return true;
}

static bool sendEv(uint8_t down) {
  if (!g_connected || !g_cmd) return false;
  bool ok = g_cmd->writeValue(&down, 1, false);
  Serial.printf("-> %s %s\n", down ? "DOWN" : "UP", ok ? "ok" : "ERROR");
  return ok;
}

static void sleepNow() {
  Serial.println("durmiendo (despierta con el botón)");
  Serial.flush();
  if (g_client && g_client->isConnected()) g_client->disconnect();
  uint32_t tw = millis();
  while (btnDown() && millis() - tw < 3000) delay(5);  // esperar a soltar: con el botón apretado se despertaría al instante
  delay(30);
  // Botón atascado en LOW (¿mal cableado?): despertar por flanco a HIGH en vez de gastar la pila en un bucle de arranque.
  bool stuck = btnDown();
  if (stuck) Serial.println("AVISO: GPIO3 sigue en LOW tras 3 s (botón atascado o mal cableado)");
  esp_deep_sleep_enable_gpio_wakeup(1ULL << PIN_BTN, stuck ? ESP_GPIO_WAKEUP_GPIO_HIGH : ESP_GPIO_WAKEUP_GPIO_LOW);
  esp_deep_sleep_start();
}

void setup() {
  Serial.begin(115200);
  uint32_t t0 = millis();
  g_t0 = t0;
  g_evq = xQueueCreate(32, sizeof(Ev));
  pinMode(PIN_BTN, INPUT_PULLUP);
  bool wokeByBtn = esp_sleep_get_wakeup_cause() == ESP_SLEEP_WAKEUP_GPIO;
  Serial.printf("\nT60 remoto %s, arranque por %s, GPIO3=%s\n", FW_VERSION, wokeByBtn ? "botón" : "encendido/reset",
                btnDown() ? "LOW (apretado)" : "HIGH (suelto)");
  // El pulso que nos despertó ya está en curso: es el 1.er DOWN, con ts=0.
  if (btnDown()) pushEv(0, 1);
  xTaskCreate(btnTask, "btn", 2048, nullptr, 2, nullptr);

  g_prefs.begin("remote", false);
  BLEDevice::init("T60 remoto");  // sin nvs_flash_erase(): el bond debe sobrevivir al deep sleep
  BLESecurity *sec = new BLESecurity();
  sec->setPassKey(true, BLE_REMOTE_PIN);
  sec->setAuthenticationMode(true, true, true);  // bonding + MITM + Secure Connections
  sec->setCapability(ESP_IO_CAP_IN);
  BLEDevice::setSecurityCallbacks(new SecCb());

  bool linked = false;
  while (!linked && millis() - t0 < CONNECT_BUDGET_MS) {
    linked = connectToTembed();
    if (!linked) delay(300);
  }
  if (!linked) { Serial.println("sin enlace: a dormir"); sleepNow(); }

  // Reproducir los eventos de antes de estar conectado con su tiempo relativo (la máquina del T-Embed cuenta los toques con
  // SU reloj y una ventana de 300 ms); los posteriores salen en loop() al instante.
  uint32_t tConn = millis() - t0, base = millis(), first = 0;
  bool haveFirst = false;
  Ev e;
  while (xQueuePeek(g_evq, &e, 0) == pdTRUE && e.ts <= tConn) {
    xQueueReceive(g_evq, &e, 0);
    if (!haveFirst) { first = e.ts; haveFirst = true; }
    while (millis() - base < e.ts - first) delay(1);
    sendEv(e.down);
  }
}

void loop() {
  static uint32_t tLast = millis(), tLevel = millis();
  Ev e;
  while (xQueueReceive(g_evq, &e, 0) == pdTRUE) {
    tLast = millis();
    sendEv(e.down);
  }
  if (!g_connected) {  // el enlace cayó con el botón en uso: un intento de reconexión corto, si no a dormir
    Serial.println("enlace perdido: reconectando");
    if (!connectToTembed()) sleepNow();
    tLast = millis();
  }
  if (millis() - tLevel > 2000) {  // diagnóstico: nivel del pin (un botón mal cableado puede leerse siempre LOW)
    tLevel = millis();
    Serial.printf("GPIO3=%s\n", btnDown() ? "LOW" : "HIGH");
  }
  if (btnDown()) tLast = millis();  // con el botón apretado no se duerme: así el UP siempre sale al soltar
  if (millis() - tLast > AWAKE_MS) sleepNow();
  delay(2);
}
