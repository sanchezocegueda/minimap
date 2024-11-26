# Finalize Interfaces with hardware
We need to get simple demo programs for all peripherals working. This means we have been able to successfully connect the hardware to the specified [GPIO pins](hardware_artifacts/esp32_pinout.jpg). *Ideally these are all RTOS tasks*

## Screen
- [x] basic esp-idf example code running
- [x] play around with [LVGL](https://docs.lvgl.io/master/intro/introduction.html)
- [x] simple 2 coordinates displayed with one centered
**- [ ] DEMO: Update screen with IMU and GPS data periodically**
**- [ ] API: GPS -- given latitude and longitude from GPS, track around a fixed point**
**- [ ] API: -- given an angle, display where North is**
- [ ] IMU -- given an angle, rotate actors on the screen
    - Rotate actors? or create vectors toward?
- [ ] Cosmetic: add radar lines

## IMU
- [x] Compile a driver and/or library
    - Decide on a library from: [here](https://github.com/hideakitai/MPU9250), or [here](https://git.sr.ht/~truita/esp-mpu9250/tree) (or if you find a better one)
    - 
- [ ] [IN PROGRESS] Convert library to use interrupts to wakeup the task
- [ ] Basic IMU data display on screen
- [ ] Calculate heading

## LORA
- [x] simple_radiolib example working
### Around Cory Hall
- [x] counter example
- [x] interrupt counter and GPS example
- [ ] use `mbedtls/aes.h` to encrypt a 16 byte packet containing 3 floats. Probably with CBC and IV?
### More Testing

## GPS
- [x] port a library to esp-idf and create a pin header for the GPS module we currently have
- [x] Try the esp-idf NMEA0183_parser
- [ ] Decide how to integrate the gps event loop into a final program

# Build initial prototype 

# Misc
- [x] solder 2 pin headers onto the other esp32 thing
- [x] resolder pin headers on the Lora Module we have??
- [x] check for gps cable ?
- [x] solder headers on lora module when it comes
- [-] we could consider trying to create even more devices to only display location with lora, iirc there are other lora boards in lab
    - There are only FSK radios...
