idf_component_register(SRCS "main.c"
                    INCLUDE_DIRS "."
                    REQUIRES ahrs mpu9250 lora nmea_parser esp_timer mbedtls)
