idf_component_register(SRCS "gps_rx_tx.c"
                    INCLUDE_DIRS "."
                    REQUIRES lora nmea_parser mbedtls)