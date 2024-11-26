# Hardware

**fill with info**

# Software

## LoRa


## IMU
Currently using [some random library](https://github.com/psiphi75/esp-mpu9250), still testing the "validity" of sensor readings
- Based on [Open source IMU and AHRS algorithms](https://x-io.co.uk/open-source-imu-and-ahrs-algorithms/)

I would like to integrate some of the code from [this](https://github.com/xioTechnologies/Fusion/tree/main), which is the current up to date library from the publisher of the paper when creating the final `MPU9250` idf component.

I'm also not confident that I want to switch entirely to interrupt based IMU access, because we will use sensor readings for power savings, it might be better to just poll it.

## GPS
The `nmea_parser` component was created from [Espressif's ESP-IDF NMEA_PARSER example code](https://github.com/espressif/esp-idf/tree/master/examples/peripherals/uart/nmea0183_parser)

## Screen

Using `lvgl`, based off [ESP-IDF's spi_master_lcd_touch example](https://github.com/espressif/esp-idf/tree/master/examples/peripherals/lcd/spi_lcd_touch)