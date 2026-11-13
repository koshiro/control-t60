#include "ring.h"

#include <BLEDevice.h>
#include <Preferences.h>
#include <esp_bt.h>
#include <esp_heap_caps.h>
#include <host/ble_gap.h>
#include <host/ble_store.h>

// esp-idf #10109: con las funciones BLE 5.0 del controlador activas, el ESP32-S3 como central no logra enlazar con
// algunos periféricos (el anillo, chip Jieli: error 0x3E en cada intento; verificado 2026-11-13: con esto apagado
// conecta en ~0.4 s, 7 de 7). Las librerías de Arduino vienen precompiladas, así que se intercepta el init del
// controlador (platformio.ini: -Wl,--wrap=esp_bt_controller_init) y se apagan ahí. El host no usa ext-adv: no pierde nada.
extern "C" esp_err_t __real_esp_bt_controller_init(esp_bt_controller_config_t *cfg);
extern "C" esp_err_t __wrap_esp_bt_controller_init(esp_bt_controller_config_t *cfg) {
  cfg->ble_50_feat_supp = false;
  return __real_esp_bt_controller_init(cfg);
}

namespace ring {

static const char *kNvs = "t60ring";
static const char *kName = "Ring01";  // nombre con que se anuncia (manual del anillo)

static TaskHandle_t s_task;
static QueueHandle_t s_q;
static BLEClient *s_client;
static volatile State s_state = State::Off;
static volatile PairResult s_pair = PairResult::None;
static volatile bool s_reqPair, s_reqForget, s_haveAddr;
static volatile bool s_want, s_up, s_expected;  // BLE pedido por el firmware · controlador encendido · corte propio
static volatile uint32_t s_lost;
static volatile int s_chars;
static String s_addr;
static uint8_t s_addrType;
struct Seen { char addr[18]; char name[20]; int8_t rssi; uint8_t type; bool hid; uint8_t adv; bool conn; };
static Seen s_seen[24];
static volatile int s_seenN;
static uint8_t s_ringAdv[62];
static volatile int s_ringAdvN;

struct Cb : BLEClientCallbacks {
  void onConnect(BLEClient *) override {}
  void onDisconnect(BLEClient *) override {
    if (!s_expected) s_lost = s_lost + 1;  // un corte nuestro (al desarmar) no es una caída
    s_chars = 0;
    if (s_state == State::Ready) s_state = State::Scanning;
  }
  bool onConnParamsUpdateRequest(BLEClient *, const ble_gap_upd_params *) override { return true; }
};
static Cb s_cb;

// Handles de las características HID Report suscritas (para filtrar las notificaciones) y resultado de cada CCCD.
static uint16_t s_repHandle[8];
static volatile int s_repN, s_cccdOk, s_cccdDone;

static int cccdCb(uint16_t, const struct ble_gatt_error *err, struct ble_gatt_attr *, void *) {
  if (err->status == 0) s_cccdOk = s_cccdOk + 1;
  s_cccdDone = s_cccdDone + 1;
  return 0;
}

static void saveAddr() {
  Preferences p;
  if (!p.begin(kNvs, false)) return;
  p.putString("addr", s_addr);
  p.putUChar("atype", s_addrType);
  p.end();
}

// Conecta, espera el cifrado (emparejar la primera vez; con la clave guardada después) y se suscribe a los reportes.
static PairResult connectTo(BLEAddress addr, uint8_t type) {
  s_state = State::Connecting;
  if (!s_client->connect(addr, type, 8000)) return PairResult::ConnectFail;
  // Orden que tolera el anillo: enlace → descubrir servicios → emparejar (legacy, Just Works) → suscribir.
  // Iniciar el cifrado apenas conecta (lo que hace la librería por defecto) lo dejaba mudo: enc_change=7 y timeout.
  vTaskDelay(pdMS_TO_TICKS(300));
  if (s_client->isConnected()) s_client->getServices();
  if (s_client->isConnected()) {
    int rc = -1;
    BLESecurity::startSecurity(s_client->getConnId(), &rc);
    for (int i = 0; i < 100 && s_client->isConnected(); i++) {
      ble_gap_conn_desc d;
      if (ble_gap_conn_find(s_client->getConnId(), &d) == 0 && d.sec_state.encrypted) break;
      vTaskDelay(pdMS_TO_TICKS(100));
    }
  }
  // Suscripción SIN la librería: su subscribe() descubre y guarda todos los descriptores (~12 KB por característica)
  // y agotaba la RAM. El CCCD de cada Report va justo después del valor (handle + 1): se escribe directo con NimBLE.
  int n = 0;
  s_repN = 0;
  s_cccdOk = s_cccdDone = 0;
  BLERemoteService *hid = s_client->isConnected() ? s_client->getService(BLEUUID((uint16_t)0x1812)) : nullptr;
  if (hid) {
    static const uint8_t on[2] = {0x01, 0x00};
    for (auto &kv : *hid->getCharacteristicsByHandle()) {
      BLERemoteCharacteristic *c = kv.second;
      if (!c->getUUID().equals(BLEUUID((uint16_t)0x2A4D)) || !c->canNotify() || s_repN >= 8) continue;  // Report de entrada
      uint16_t h = c->getHandle();
      s_repHandle[s_repN] = h;
      s_repN = s_repN + 1;
      if (ble_gattc_write_flat(s_client->getConnId(), h + 1, on, sizeof on, cccdCb, nullptr) == 0) n++;
    }
    for (int i = 0; i < 50 && s_cccdDone < n && s_client->isConnected(); i++) vTaskDelay(pdMS_TO_TICKS(100));
    n = s_cccdOk;
  }
  Serial.printf("RING conectado=%d reportes HID suscritos=%d de %d (heap %u)\n", s_client->isConnected(), n, (int)s_repN,
                (unsigned)heap_caps_get_free_size(MALLOC_CAP_INTERNAL));
  if (n == 0 || !s_client->isConnected()) {
    bool was = s_client->isConnected();
    if (was) s_client->disconnect();
    return hid || !was ? PairResult::ConnectFail : PairResult::NoHid;
  }
  s_chars = n;
  return PairResult::Ok;
}

// Informa por serial el motivo de cada desconexión y el resultado del cifrado / emparejado.
static ble_gap_event_listener s_listener;
static int gapSpy(struct ble_gap_event *e, void *) {
  switch (e->type) {
    case BLE_GAP_EVENT_DISCONNECT: Serial.printf("RING desconectado (motivo 0x%X)\n", e->disconnect.reason); break;
    case BLE_GAP_EVENT_ENC_CHANGE: Serial.printf("RING cifrado status=0x%X\n", e->enc_change.status); break;
    case BLE_GAP_EVENT_PARING_COMPLETE: Serial.printf("RING emparejado status=0x%X\n", e->pairing_complete.status); break;
    case BLE_GAP_EVENT_NOTIFY_RX: {  // reporte HID: directo a la cola (sin pasar por los mapas de la librería)
      bool mine = false;
      for (int i = 0; i < s_repN; i++) mine = mine || s_repHandle[i] == e->notify_rx.attr_handle;
      if (!mine || !s_q) break;
      Report r;
      r.t = micros();
      r.src = e->notify_rx.attr_handle;
      int len = OS_MBUF_PKTLEN(e->notify_rx.om);
      r.len = (uint8_t)(len > (int)sizeof r.data ? sizeof r.data : len);
      os_mbuf_copydata(e->notify_rx.om, 0, r.len, r.data);
      xQueueSend(s_q, &r, 0);
      break;
    }
    default: break;
  }
  return 0;
}

// Enciende el BLE (controlador + host + cliente). Se llama al armar / abrir el menú / emparejar.
static bool bleUp() {
  if (!BLEDevice::init("control-t60")) return false;  // sin RAM para el controlador
  ble_gap_event_listener_register(&s_listener, gapSpy, nullptr);
  BLESecurity::setAuthenticationMode(true, false, false);  // bonding, sin MITM ni SC (el anillo no tiene pantalla)
  BLESecurity::setForceAuthentication(false);              // la seguridad la inicia connectTo, después de descubrir
  BLESecurity::setCapability(ESP_IO_CAP_NONE);
  s_client = BLEDevice::createClient();
  if (!s_client) { BLEDevice::deinit(false); return false; }
  s_client->setClientCallbacks(&s_cb);
  s_expected = false;
  return true;
}

// Apaga el BLE ENTERO (controlador incluido): desarmado el anillo no puede hacer nada, así que no se gasta radio ni
// se impide el light sleep (el controlador no sobrevive dormido). Las claves de bonding quedan en NVS.
static void bleDown() {
  s_expected = true;
  if (s_client && s_client->isConnected()) {
    s_client->disconnect();
    for (int i = 0; i < 40 && s_client->isConnected(); i++) vTaskDelay(pdMS_TO_TICKS(25));  // esperar el corte limpio
  }
  ble_gap_event_listener_unregister(&s_listener);
  BLEDevice::deinit(false);  // borra también el cliente
  s_client = nullptr;
  s_chars = 0;
  s_repN = 0;
}

static void task(void *) {
  uint32_t searchStart = 0;
  for (;;) {
    bool need = (s_want && s_haveAddr) || s_reqPair;
    if (s_reqForget) {
      if (s_up) {
        s_expected = true;
        if (s_client->isConnected()) s_client->disconnect();
        ble_store_clear();
      }
      { Preferences p; if (p.begin(kNvs, false)) { p.remove("addr"); p.remove("atype"); p.end(); } }
      s_haveAddr = false;
      s_addr = "";
      s_reqForget = false;
      continue;
    }
    if (!need) {  // nada que hacer: BLE apagado del todo y la tarea bloqueada hasta el próximo pedido
      if (s_up) { bleDown(); s_up = false; }
      s_state = State::Off;
      ulTaskNotifyTake(pdTRUE, portMAX_DELAY);
      continue;
    }
    if (!s_up) {
      if (!bleUp()) {
        if (s_reqPair) { s_pair = PairResult::NoBle; s_reqPair = false; }
        s_state = State::Off;
        ulTaskNotifyTake(pdTRUE, pdMS_TO_TICKS(5000));
        continue;
      }
      s_up = true;
      searchStart = millis();
    }
    BLEScan *scan = BLEDevice::getScan();
    if (s_client->isConnected() && !s_reqPair) {  // enlazado: nada que hacer más que vigilar pedidos
      s_state = State::Ready;
      searchStart = millis();
      ulTaskNotifyTake(pdTRUE, pdMS_TO_TICKS(500));
      continue;
    }
    if (s_client->isConnected()) { s_client->disconnect(); vTaskDelay(pdMS_TO_TICKS(300)); }

    if (s_reqPair) {  // emparejar: escaneo activo (el nombre puede venir en el scan response), el más cercano gana
      s_state = State::PairScan;
      scan->setActiveScan(true);
      scan->setInterval(100);
      scan->setWindow(90);
      // Tandas de 2 s (hasta 8 s): conectar apenas aparece, mientras el anillo sigue anunciándose.
      BLEScanResults *res = nullptr;
      for (int round = 0; round < 4; round++) {
        res = scan->start(2, round > 0);
        bool any = false;
        for (int i = 0; res && i < res->getCount() && !any; i++) {
          String nm = res->getDevice(i).getName();
          nm.toLowerCase();
          any = nm.startsWith("ring");
        }
        if (any) break;
      }
      // Candidato: se llama "Ring…" o, si ninguno, anuncia el servicio HID / apariencia HID. Gana el más cercano.
      int best = -1, bestRssi = -999, bestRank = 0;
      s_seenN = 0;
      for (int i = 0; res && i < res->getCount(); i++) {
        BLEAdvertisedDevice d = res->getDevice(i);
        String nm = d.haveName() ? d.getName() : String("");
        bool hid = d.isAdvertisingService(BLEUUID((uint16_t)0x1812)) ||
                   (d.haveAppearance() && (d.getAppearance() & 0xFFC0) == 0x03C0);
        if (s_seenN < 24) {
          Seen &s = s_seen[s_seenN];
          strlcpy(s.addr, d.getAddress().toString().c_str(), sizeof s.addr);
          strlcpy(s.name, nm.c_str(), sizeof s.name);
          s.rssi = (int8_t)d.getRSSI();
          s.type = d.getAddressType();
          s.hid = hid;
          s.adv = d.getAdvType();
          s.conn = d.isConnectable();
          s_seenN = s_seenN + 1;
        }
        if (nm.startsWith("Ring")) {
          s_ringAdvN = min((int)d.getPayloadLength(), (int)sizeof s_ringAdv);
          memcpy(s_ringAdv, d.getPayload(), s_ringAdvN);
        }
        String low = nm;
        low.toLowerCase();
        int rank = low.startsWith("ring") ? 2 : hid ? 1 : 0;
        if (rank == 0 || rank < bestRank) continue;
        if (rank > bestRank || d.getRSSI() > bestRssi) { bestRank = rank; bestRssi = d.getRSSI(); best = i; }
      }
      PairResult pr = PairResult::NotFound;
      if (best >= 0) {
        BLEAdvertisedDevice d = res->getDevice(best);
        BLEAddress a = d.getAddress();
        uint8_t type = d.getAddressType();
        scan->clearResults();
        ble_store_clear();  // un solo anillo: claves viejas fuera (si el anillo fue reseteado, no sirven)
        for (int attempt = 0; attempt < 3 && pr != PairResult::Ok && !s_reqForget; attempt++) {  // enlace débil: reintentar
          if (attempt) vTaskDelay(pdMS_TO_TICKS(500));
          pr = connectTo(a, type);
        }
        if (pr == PairResult::Ok) {
          s_addr = a.toString();
          s_addrType = type;
          s_haveAddr = true;
          saveAddr();
        }
      } else {
        scan->clearResults();
      }
      s_pair = pr;
      s_reqPair = false;
      continue;
    }

    // Reconexión: escaneo pasivo hasta ver anunciarse al anillo emparejado. Los primeros 20 s (recién armado o
    // recién caído el enlace) a ciclo alto para conectar rápido; después 40 ms cada 500 ms (8 %) para no gastar
    // batería buscando un anillo apagado.
    s_state = State::Scanning;
    bool fast = millis() - searchStart < 20000;
    scan->setActiveScan(false);
    scan->setInterval(fast ? 100 : 500);
    scan->setWindow(fast ? 60 : 40);
    BLEScanResults *res = scan->start(fast ? 1 : 3, false);
    BLEAddress want(s_addr, s_addrType);
    bool seen = false;
    for (int i = 0; res && i < res->getCount() && !seen; i++) seen = res->getDevice(i).getAddress() == want;
    scan->clearResults();
    if (seen && s_want && !s_reqPair && !s_reqForget && connectTo(want, s_addrType) != PairResult::Ok)
      vTaskDelay(pdMS_TO_TICKS(500));
  }
}

static void ensureTask() {
  if (!s_q) s_q = xQueueCreate(64, sizeof(Report));
  if (!s_task) xTaskCreatePinnedToCore(task, "t60ring", 8192, nullptr, 2, &s_task, 0);
  else xTaskNotifyGive(s_task);
}

void begin() {
  Preferences p;
  if (p.begin(kNvs, true)) {
    s_addr = p.getString("addr", "");
    s_addrType = p.getUChar("atype", 0);
    p.end();
  }
  s_haveAddr = s_addr.length() == 17;  // el BLE no se enciende aquí: lo pide setActive() al armar / abrir el menú
}

void setActive(bool on) {
  s_want = on;
  if (s_task) xTaskNotifyGive(s_task);
  else if (on && s_haveAddr) ensureTask();
}

bool isUp() { return s_up; }

void pair() {
  s_pair = PairResult::Busy;
  s_reqPair = true;
  ensureTask();
}

void forget() {
  if (!s_task) {  // BLE apagado: basta con borrar lo guardado (las claves se limpian al volver a emparejar)
    Preferences p;
    if (p.begin(kNvs, false)) { p.remove("addr"); p.remove("atype"); p.end(); }
    s_haveAddr = false;
    s_addr = "";
    return;
  }
  s_reqForget = true;
  xTaskNotifyGive(s_task);
}

State state() { return s_state; }
const char *stateName() {
  switch (s_state) {
    case State::Scanning: return "buscando";
    case State::Connecting: return "conectando";
    case State::Ready: return "conectado";
    case State::PairScan: return "emparejando";
    default: return s_haveAddr ? "en reposo (conecta al armar)" : "sin emparejar";
  }
}
PairResult pairResult() { return s_pair; }
const char *pairResultName() {
  switch (s_pair) {
    case PairResult::Busy: return "en curso";
    case PairResult::Ok: return "OK";
    case PairResult::NotFound: return "no se vio ningun Ring01";
    case PairResult::ConnectFail: return "fallo la conexion";
    case PairResult::NoHid: return "sin servicio HID";
    case PairResult::NoBle: return "BLE no inicia (RAM)";
    default: return "-";
  }
}
bool paired() { return s_haveAddr; }
String address() { return s_addr; }
int reportChars() { return s_chars; }
bool read(Report &r) { return s_q && xQueueReceive(s_q, &r, 0) == pdTRUE; }
uint32_t lostCount() { return s_lost; }
void printSeen(Print &out) {
  out.printf("RINGADV %d bytes:", (int)s_ringAdvN);
  for (int i = 0; i < s_ringAdvN; i++) out.printf(" %02X", s_ringAdv[i]);
  out.println();
  out.printf("RINGSCAN %d dispositivos en el ultimo escaneo\n", (int)s_seenN);
  for (int i = 0; i < s_seenN; i++)
    out.printf("  %s tipo=%u rssi=%d hid=%d adv=%u conectable=%d nombre='%s'\n", s_seen[i].addr, s_seen[i].type,
               s_seen[i].rssi, s_seen[i].hid, s_seen[i].adv, s_seen[i].conn, s_seen[i].name);
}

bool loadTable(ringmap::Table &t) {
  ringmap::clear(t);
  Preferences p;
  if (!p.begin(kNvs, true)) return false;
  ringmap::Table tmp;
  bool ok = p.getBytesLength("tbl") == sizeof tmp && p.getBytes("tbl", &tmp, sizeof tmp) == sizeof tmp && ringmap::valid(tmp);
  p.end();
  if (ok) t = tmp;
  return ok;
}

bool saveTable(const ringmap::Table &t) {
  Preferences p;
  if (!ringmap::valid(t) || !p.begin(kNvs, false)) return false;
  bool ok = p.putBytes("tbl", &t, sizeof t) == sizeof t;
  p.end();
  return ok;
}

}  // namespace ring
