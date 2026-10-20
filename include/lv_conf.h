// Configuración de LVGL 9 para control-t60 (T-Embed CC1101, ST7789V 320×170). Lo no definido aquí toma el
// default de lv_conf_internal.h. Docs: https://lvgl.io/docs/open
#ifndef LV_CONF_H
#define LV_CONF_H

#define LV_COLOR_DEPTH 16                  // RGB565; el flush lo pasa a LovyanGFX con swap de bytes
#define LV_USE_STDLIB_MALLOC LV_STDLIB_BUILTIN
#define LV_MEM_SIZE (48U * 1024U)          // heap propio de LVGL (RAM interna)
#define LV_USE_OS LV_OS_NONE               // todo LVGL corre en loop() (core 1); el SPI se serializa con rfLock
#define LV_DEF_REFR_PERIOD 30
#define LV_DPI_DEF 160

#define LV_USE_LOG 0
#define LV_USE_ASSERT_NULL 1
#define LV_USE_ASSERT_MALLOC 1

#define LV_FONT_MONTSERRAT_12 1
#define LV_FONT_MONTSERRAT_14 1
#define LV_FONT_MONTSERRAT_20 1
#define LV_FONT_DEFAULT &lv_font_montserrat_14

#define LV_USE_THEME_DEFAULT 1
#define LV_THEME_DEFAULT_DARK 1
#define LV_THEME_DEFAULT_GROW 0            // sin animación de "crecer" al enfocar (pantalla chica)

#define LV_BUILD_EXAMPLES 0
#define LV_BUILD_DEMOS 0

#endif
