idf_component_register(SRCS "bt_a2dp.c" "bt_app_core.c" "bt_app_av.c"
                    INCLUDE_DIRS "include"
                    PRIV_REQUIRES esp_driver_i2s bt nvs_flash esp_ringbuf esp_driver_dac)
