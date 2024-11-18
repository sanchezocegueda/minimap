idf_component_register(SRCS "hardware_integration.cpp"
                    INCLUDE_DIRS "."
                    REQUIRES ahrs mpu9250 RadioLib nmea_parser esp_timer)