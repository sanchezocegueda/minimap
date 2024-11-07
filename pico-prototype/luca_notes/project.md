# Links
[BOM](https://docs.google.com/spreadsheets/d/15W4wwxXkQQS-kHKKcw-9356XjkgM5HsGhcNUo7VEne0/edit?gid=1277098881#gid=1277098881)
[Part Request Form](https://docs.google.com/forms/d/e/1FAIpQLScOeeJ1YqKZxFd3QCQ9UUkvl_Z1RGB9qkjTkKaBnJdHM-YsBA/formResponse)
[Lab Inventory](https://docs.google.com/spreadsheets/d/1v2LrFACQgDAR7JVOlNeXz1KoTylSGIU5DAnORZ1ogkU/edit?gid=0#gid=0)

---
# Battery
- 4.2 - 3.7V based on charge.
- max current output: 

Powering a nano with 3.7V
- <https://forum.arduino.cc/t/arduino-nano-3-7v-lipo-battery/948705/12>
- <https://forum.arduino.cc/t/powering-arduino-nano-with-3-7-v-li-ion-cell/1194977/4>

---
# [RFM9x](https://www.adafruit.com/product/3072)
[pinouts](https://learn.adafruit.com/adafruit-rfm69hcw-and-rfm96-rfm95-rfm98-lora-packet-padio-breakouts/pinouts)
[radiolib arduino test](https://learn.adafruit.com/adafruit-rfm69hcw-and-rfm96-rfm95-rfm98-lora-packet-padio-breakouts/rfm9x-test)
- Arduino library seems ok? No explicit support for Arduino Nanos...

---
# [ESP32](https://www.sparkfun.com/products/20168)
[hardware overview](https://learn.sparkfun.com/tutorials/esp32-thing-plus-usb-c-hookup-guide/hardware-overview)
- 3.7 battery should be just fine for these

---
# mpu9250 IMU library
<https://github.com/hideakitai/MPU9250/tree/master>

---
# mtk3339 GPS library
<https://github.com/stealthylabs/libgps_mtk3339/tree/master/src>

---
# TODO
- [ ] solder header on IMU
- [ ] resolder header on lora board
- [ ] verify IMUs
- [ ] verify batteries and battery charger
- [ ] try radiolib with nano
- [ ] try radiolib with esp32
- [ ] try wolfssl with nano
- [ ] try wolfssl with esp32

# Goals
- Transmit IMU data through lora radio
