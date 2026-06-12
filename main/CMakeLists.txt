idf_component_register(SRCS "main.c"
                    PRIV_REQUIRES nvs_flash driver sht30 ssd1306 wifi mqtt json ota watchdog can
                    INCLUDE_DIRS ".")
