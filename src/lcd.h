// Pantalla ST7789V 320×170 del T-Embed CC1101 por LovyanGFX. Pines: notas LilyGO (utilities.h) + control-firmware.md.
// El bus es FSPI, COMPARTIDO con el CC1101 (CS 12), la SD (13) y el nRF24 (44). LovyanGFX en el host por defecto
// (SPI2) hace `SPI.end(); SPI.begin(sclk, miso, mosi)` sobre el `SPI` global de Arduino y restaura el registro
// USER (full duplex) en cada endTransaction → el CC1101 debe usar ese MISMO `SPI` y el LCD se inicializa antes.
// Serializar siempre: nunca dibujar con una TX en curso (rfLock) ni a la vez que un acceso al CC1101 (g_spiLock).
#pragma once
#define LGFX_USE_V1
#include <LovyanGFX.hpp>

#define PIN_LCD_DC 16
#define PIN_LCD_BL 21

class LcdT60 : public lgfx::LGFX_Device {
  lgfx::Panel_ST7789 panel_;
  lgfx::Bus_SPI bus_;

 public:
  LcdT60() {
    auto b = bus_.config();
    b.spi_host = SPI2_HOST;  // = FSPI, el del CC1101
    b.spi_mode = 0;
    b.freq_write = 40000000;
    b.freq_read = 16000000;
    b.spi_3wire = false;
    b.use_lock = true;   // toma el mutex HAL de Arduino, el mismo que SPI.beginTransaction
    b.dma_channel = 0;   // sin DMA: escrituras por FIFO, terminan dentro de cada llamada
    b.pin_sclk = 11;
    b.pin_mosi = 9;
    b.pin_miso = 10;     // LovyanGFX re-inicia el SPI global con estos pines: el CC1101 necesita MISO
    b.pin_dc = PIN_LCD_DC;
    bus_.config(b);
    panel_.setBus(&bus_);

    auto p = panel_.config();
    p.pin_cs = 41;
    p.pin_rst = -1;      // no conectado (LilyGO: DISPLAY_RST -1; el 40 es LRCLK del audio)
    p.pin_busy = -1;
    p.panel_width = 170;
    p.panel_height = 320;
    p.memory_width = 240;
    p.memory_height = 320;
    p.offset_x = 35;     // "gap Y=35" en landscape = columna 35 de la RAM en vertical
    p.offset_y = 0;
    p.offset_rotation = 0;
    p.readable = false;
    p.invert = true;
    p.rgb_order = false;
    p.dlen_16bit = false;
    p.bus_shared = true;
    panel_.config(p);
    setPanel(&panel_);
  }
};
