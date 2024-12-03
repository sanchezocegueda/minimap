# Hardware

**fill with info**

# Software
The goal is to eventually integrate everything into `hardware_integration`, but each `examples` section references working demos for each library.
- This should have the demo of 2 devices communicating and receiving GPS data and displaying their
coordinates on the screen (and future improvements made to this).

*old examples can be found in [.old](.old), but I don't recommend looking at them unless you want to see more esp-idf code that works on our setup.*

## LoRa
Using our modification of [some random, small, simple, C library](https://github.com/Inteform/esp32-lora-library).
- So far support has been added for interrupts with `lora_send/receive_packet_blocking()` in `lora.h`, but the default API will work as intended.
- Maybe interrupts are unecessary for transmitting messages?

### Examples
[lora-library](lora-library) is the project to test all lora communication.

## IMU
Currently using [some random library](https://github.com/psiphi75/esp-mpu9250), still testing the "validity" of sensor readings.
- Pitch and roll do not seem correct, but we need to take a closer look and try to calibrate it differently.
- Based on [Open source IMU and AHRS algorithms](https://x-io.co.uk/open-source-imu-and-ahrs-algorithms/).

Would like to integrate some of the code from [the Fusion library](https://github.com/xioTechnologies/Fusion/tree/main), which is the current up to date library from the publisher of the paper, when creating the final `MPU9250` idf component.

### Interrupts
Not confident that we want to switch entirely to interrupt based IMU access. Because we will use sensor readings for power savings, it might be better to just poll it (less context switching because we have to handle interrupts).

So far there is some some demo code in [imu_irq_in_progress](imu_irq_in_progress). The code flashes but does not seem to interrupt as often as it should be, it could be because the register configuration for interrupt support in esp-idf on the MPU9250 is not correct, see [imu_interrupts](imu_irq_in_progress/notes/imu_interrupts.md).

### Examples
- [esp-mpu9250](esp-mpu9250) is the project example that came with the library.
- [screen_imu_gps](screen_imu_gps) is a project with the `nmea_parser` component and the imu library integrated as components: `mpu9250` and `ahrs`.

## GPS
The `nmea_parser` component was created from [Espressif's ESP-IDF NMEA_PARSER example code](https://github.com/espressif/esp-idf/tree/master/examples/peripherals/uart/nmea0183_parser)

### Examples
- [nmea0183_parser](nmea0183_parser) is the ESP-IDF example project.
- Also see [screen_imu_gps](screen_imu_gps) and [lora-library](lora-library) for more examples of integrating the parser component into projects.

## Screen
Using `lvgl`, based off [ESP-IDF's spi_master_lcd_touch example](https://github.com/espressif/esp-idf/tree/master/examples/peripherals/lcd/spi_lcd_touch)

### Examples
- [spi_lcd_touch](spi_lcd_touch) is the original example Varun and Alex started their screen work on.
    - Alex and Luca began simplifying this into [screen_imu_gps](screen_imu_gps)


## Application Design

## GPS: nmea_parser
```c
nmea_parser_handle_t nmea_parser_init(const nmea_parser_config_t *config)
```
In this function  we setup the UART driver, which also sets up pattern detection to create a software interrupt when the `\n` character is sent from the GPS.

Specifically, when a pattern is detected, `esp_handle_uart_pattern` is called.
- This calls `gps_decode` on the pattern, which does the actual parsing from the hardware.

It creates the `esp_gps->event_queue` that is updated when uart detects a pattern or encounters an error and also creates a task running `nmea_parser_task_entry`, this is the task that blocks on the `esp_gps->event_queue` and calls `esp_handle_uart_pattern` to parse a pattern. Once all our statements are parsed

This also creates an event loop for components to register event handlers with.

```c
esp_err_t nmea_parser_add_handler(nmea_parser_handle_t nmea_hdl, esp_event_handler_t event_handler, void *handler_args, nmea_event_id_t event_id)
```
Allows components to register functions when events are posted from the `nmea_parser_task_entry` task.

- `event_id` is the task that should receive (for now, `SCREEN` or `ENCRYPTION`)
- `nmea_hdl` is the return of `nmea_parser_init`
- `event_handler` is a function to run, here is an example definition
```c
static void gps_event_handler(void *event_handler_arg, esp_event_base_t event_base, int32_t event_id, void *event_data)
```
- `event_data` can be cast to `gps_t *` to process gps data