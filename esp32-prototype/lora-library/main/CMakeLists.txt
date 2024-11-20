idf_component_register(SRCS "ping-pong.c" "lora-library.c"
                    INCLUDE_DIRS "."
                    REQUIRES lora nmea_parser)
