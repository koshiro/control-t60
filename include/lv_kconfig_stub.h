// Stub para LV_CONF_KCONFIG_EXTERNAL_INCLUDE: sin esto lv_conf_kconfig.h incluye esp_attr.h también al ensamblar
// los .S de LVGL (helium, solo ARM), donde PlatformIO no pasa las rutas de IDF. control-t60 no usa el Kconfig de LVGL.
#pragma once
