# Links
[BOM](https://docs.google.com/spreadsheets/d/15W4wwxXkQQS-kHKKcw-9356XjkgM5HsGhcNUo7VEne0/edit?gid=1277098881#gid=1277098881)
[Part Request Form](https://docs.google.com/forms/d/e/1FAIpQLScOeeJ1YqKZxFd3QCQ9UUkvl_Z1RGB9qkjTkKaBnJdHM-YsBA/formResponse)
[Lab Inventory](https://docs.google.com/spreadsheets/d/1v2LrFACQgDAR7JVOlNeXz1KoTylSGIU5DAnORZ1ogkU/edit?gid=0#gid=0)

## Important for 11/7/24
- [ ] DESIGN PERIPHERAL PINOUTS FOR PICO-W AND ESP32
- [ ] PICK MICROCONTROLLER
- [ ] DECIDE ON PCB
- [ ] ORDER THE SAME GPS MODULE FOR 2ND PROTOTYPE
- [ ] DEEP RESEARCH INTO USING ESPRESSIF IDF AND/OR RTOS
- [ ] PROS AND CONS OF SOFTWARE FRAMEWORKS

# Software
## Crypto
- [wolfssl](https://www.wolfssl.com/docs/)
- [mbedutils](https://github.com/Mbed-TLS/mbedtls)
    - Note this is included in pico-sdk as well as esp32-idf
---
## Microcontrollers
### ESP32's
[Espressif IDF](https://docs.espressif.com/projects/esp-idf/en/stable/esp32/get-started/index.html)
- RTOS for scheduling and threads
- Hardest dev environment by far
- Either using 2 [SparkFun Thing Plus - esp32-WROOM](https://www.sparkfun.com/products/20168) or 2 [SparkFun esp32 Thing](https://www.sparkfun.com/products/13907)
- I thought maybe these chips would be different when I saw them in lab, but they are super similar. After looking further it seems like the esp32-WROOM is a better choice for 4x bigger flash memory.

Alternatively, we can use Arduino IDE with esp32 boards.
- Really easy to flash some code on, probably significantly harder to create the final product.

### Pico's
The picos will also work just as easily in Arduino IDE, but suffer the same problem.

Alternatively, I have created a basic Cmake project based around the pico-sdk and picotool (similar to how lab was run). So we could use VSCode and Cmake to do all of the work.

## Notes on the PCB hardware in general
Using an STM32L4 chip for the pcb, will make it extremely hard to integrate with any of our boards unless we switch to some STM toolkit ASAP, but it's very unlikely we can get 2 boards in time to make meaningful work.

This leaves us between the rp2040 and the esp32, which should be selected to match the dev board we choose.

To be 100% honest, I'm not opposed to ditching the PCB entirely. I think it would free up Nick's time for other stuff and avoid the headache of having to manage drastically different hardware peripherals.

---
## Peripheral Libraries

### LoRa
Library: [RadioHead](http://www.airspayce.com/mikem/arduino/RadioHead/)
- Manager-Driver framework
- Essentially has radio versions of TCP and UDP frames
- Driver object is the representation of the underlying radio (RFM9X), in our case, the `RH_RF95`
- Manager is responsible for the delivery of messages and has a Driver object

Driver: [RH_RF95](http://www.airspayce.com/mikem/arduino/RadioHead/classRH__RF95.html)
Manager: [RHDatagram](http://www.airspayce.com/mikem/arduino/RadioHead/classRHDatagram.html)
- "UDP Frame"
Manager: [RHReliableDatagram](http://www.airspayce.com/mikem/arduino/RadioHead/classRHReliableDatagram.html)
- "TCP Frame"

Library: [RadioLib](https://github.com/jgromes/RadioLib)
- Lower level library that mostly handles the drivers afaik. TBH, I'm not sure we'd want to use this unless we are going RTOS route. I think using the RadioLib `RHDatagram` for broadcasting our data and `RHReliableDatagram` for broadcasting pairing information would be really straightforward, although the way this library works might make it extremely challenging to do our power saving message transmit protocol we were discussing because.
- 
### GPS
Need to find one for the GPS I picked up today.

The GPS on the PCB may be able to use [this library](https://github.com/stealthylabs/libgps_mtk3339/tree/master/src) 

# mpu9250 IMU library
<https://github.com/hideakitai/MPU9250/tree/master>
- If we have no troubles with the library, in theory the IMU should be super easy to setup and interface with.
- This is a small library














---
Not really important information

---
# Battery
- 4.2 - 3.7V based on charge.
- max current output: 

---
# [RFM9x](https://www.adafruit.com/product/3072)
[Adafruit guide](https://learn.adafruit.com/adafruit-rfm69hcw-and-rfm96-rfm95-rfm98-lora-packet-padio-breakouts)

---
# [ESP32](https://www.sparkfun.com/products/20168)
[hardware overview](https://learn.sparkfun.com/tutorials/esp32-thing-plus-usb-c-hookup-guide/hardware-overview)

---
# TODO
- [ ] solder header on IMU
- [ ] resolder header on lora board
- [ ] verify IMUs
- [ ] verify batteries and battery charger
- [ ] Try radiohead
    - [ ] We could get one pico to transmit RF_Datagrams pretty quickly.
- [ ] try mbedutils
- [ ] try wolfssl
