idf_component_register(SRCS "screen_demo.c" "main.c" # "lvgl_demo_ui.c" 
                     INCLUDE_DIRS "."
                    	REQUIRES ahrs mpu9250 nmea_parser esp_timer)
