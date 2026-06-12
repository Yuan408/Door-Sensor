idf_component_register(SRCS "wifi_sta.c"
                    REQUIRES esp_wifi esp_event nvs_flash
                    INCLUDE_DIRS ".")