# Finalize Interfaces with hardware
We need to get simple demo programs for all peripherals working. This means we have been able to successfully connect the hardware to the specified [GPIO pins](hardware_artifacts/esp32_pinout.jpg). *Ideally these are all RTOS tasks*

## Screen
- [ ] basic esp-idf example code running
- [ ] play around with [LVGL](https://docs.lvgl.io/master/intro/introduction.html)
- [ ] simple 2 coordinates displayed with one centered
- [ ] implementing arrow from center given GPS heading

## IMU
- [ ] Compile a driver and/or library
- [ ] Basic IMU data display
- [ ] Calculate heading

## LORA
- [ ] simple_radiolib example working, verified with Oscilloscope

## GPS
- [ ] port a library to esp-idf and create a pin header for the GPS module we currently have

# Build initial prototype 

# Misc
- [ ] solder 2 pin headers onto the other esp32 thing
- [ ] resolder pin headers on the Lora Module we have??