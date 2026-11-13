// Enlace BLE con el anillo Ring01 (control remoto HID de 3 botones). El T-Embed es el CENTRAL: busca el anillo, se
// empareja (Just Works, con bonding), se suscribe a todos los reportes HID de entrada y los entrega crudos por una
// cola. Qué botón es cada reporte lo decide lib/ringmap (firmas aprendidas); qué estímulo sale, lib/gesture.
// El BLE solo está encendido mientras hace falta: equipo ARMADO (el anillo solo actúa armado), menú abierto o
// emparejando. El resto del tiempo el controlador queda apagado: cero consumo de radio y permite el light sleep.
#pragma once
#include <Arduino.h>

#include "ringmap.h"

namespace ring {

enum class State : uint8_t { Off, Scanning, Connecting, Ready, PairScan };
enum class PairResult : uint8_t { None, Busy, Ok, NotFound, ConnectFail, NoHid, NoBle };

struct Report {
  uint32_t t;        // micros() al llegar
  uint16_t src;      // handle de la característica HID Report
  uint8_t len;       // bytes guardados en data (≤ sizeof data)
  uint8_t data[16];
};

void begin();              // lee la dirección guardada (no enciende el BLE: eso lo pide setActive)
void setActive(bool on);    // true: BLE encendido y buscando/conectado al anillo; false: BLE apagado del todo
bool isUp();               // controlador BLE encendido (no se puede entrar a light sleep mientras lo esté)
void pair();               // buscar un "Ring01" anunciándose, emparejar y guardarlo (asíncrono: ver pairResult)
void forget();             // olvidar el anillo (dirección + claves de bonding) y desconectar
State state();
const char *stateName();
PairResult pairResult();
const char *pairResultName();
bool paired();
String address();
int reportChars();         // características HID suscritas en la conexión actual
bool read(Report &r);      // siguiente reporte recibido (llamar desde loop)
uint32_t lostCount();      // sube con cada desconexión
void printSeen(Print &out); // lo que vio el último escaneo de emparejado (diagnóstico)

bool loadTable(ringmap::Table &t);  // firmas de botones en NVS; false (y tabla vacía) si no hay o está corrupta
bool saveTable(const ringmap::Table &t);

}  // namespace ring
