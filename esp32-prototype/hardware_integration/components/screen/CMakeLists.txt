idf_component_register(SRCS "calibrate_ui.c" "screen.c" "ui.c" "calibrate_ui.c"
INCLUDE_DIRS "include"
REQUIRES esp_timer nmea_parser mpu9250 ahrs buttons)
