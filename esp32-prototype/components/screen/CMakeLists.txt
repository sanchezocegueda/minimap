idf_component_register(SRCS "screen.c" "ui.c"
INCLUDE_DIRS "include"
REQUIRES esp_timer nmea_parser mpu9250 ahrs)
