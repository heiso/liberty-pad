idf_component_register(
  SRCS "main.c"
       "sensor.c"
       "hid.c"
       "esp_hidd_prf_api.c"
       "hid_device_le_prf.c"
       "haptic.c"
  INCLUDE_DIRS "."
  PRIV_REQUIRES esp_adc bt nvs_flash esp_driver_i2c
)

target_compile_options(${COMPONENT_LIB} PRIVATE -Wno-unused-const-variable)