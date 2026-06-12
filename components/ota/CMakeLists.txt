idf_component_register(SRCS "ota_manager.c"
                    REQUIRES esp_https_ota esp_partition app_update
                    INCLUDE_DIRS ".")
