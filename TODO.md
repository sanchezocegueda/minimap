# SCREEN
- [x] basic esp-idf example code running
- [x] play around with [LVGL](https://docs.lvgl.io/master/intro/introduction.html)
- [x] simple 2 coordinates displayed with one centered
- [x] **DEMO: Update screen with IMU and GPS data periodically**
- [x] **API: GPS -- given latitude and longitude from GPS, track around a fixed point**
    - Could be refined
- [ ] **API: -- given an angle, display where North is**
    - In progress
- [ ] **Transform the calibration code to display on the screen so we can calibrate with a battery**
- [ ] Develop menu ui
- [ ] Integrate button interrupts to control menus
- [ ] Cosmetic: add radar lines


# IMU
- [x] Compile a driver and/or library
    - Decide on a library from: [here](https://github.com/hideakitai/MPU9250), or [here](https://git.sr.ht/~truita/esp-mpu9250/tree) (or if you find a better one)
- [x] Basic IMU data display on screen
- [x] **Calculate heading**
- [x] **Experiment with [the Fusion library](https://github.com/xioTechnologies/Fusion/tree/main)**
    - not needed
- [x] **Test pitch, roll, heading, and calibration more**
- [x] **Decide if/how to use interrupts to wakeup the task**
- [ ] implement writing to a thread-safe global struct
- [ ] continuous calibration


# LORA
- [x] simple receive and transmit achieved
- [x] counter example
- [x] interrupt counter and GPS example
- [ ] **use `mbedtls/aes.h` to encrypt a 16 byte packet containing 3 floats. Probably with AES-CTR and IV?**
- [x] single program rx/tx
- [ ] clock synch with ptp algorithm
- [ ] button ptp and send symmetric key


# GPS
- [x] port a library to esp-idf and create a pin header for the GPS module we currently have
- [x] Try the esp-idf NMEA0183_parser
- [x] Decide how to integrate the gps event loop into a final program
    - [ ] implement writing to a thread-safe global struct


# OTHER
- [x] Iterate on the prototype once more: have 2 breadboards with the exact same wiring
    - [ ] Add buttons and configure idf for interrupts and gpio
- [x] Cleanup components so there is only 1 copy of the source files for each example.
- [ ] Cleanup repo even more
- [ ] Demo to clock synchronize and transfer data on a button press

# TODAY
- [ ] Move lora stuff from app_main into it's on task
    - this should handle the hardware setup of the radio
- [ ] Research
    - Enable implicit header mode
    - experiment with spreading factor, bandwidth, and higher powers (20dBm)
    - Try to test these irl
