Descriptions of the pins on our peripherals that are connected to our microcontroller
- <peripheral pin> : <esp32 gpio pin>, <description>

## IMU
- SCL: 19, Serial clock
- SDA: 23, Serial in
- AD0: 18, Serial interface
- INT: 5, Interrupt pin

## Screen
- RST: 2, Reset screen
- CS: 0, SPI chip select
- DC: 4, Data or command
- SDA: 17, Serial in
- SCL: 16, Serial clock

## Lora
- EN: 25, Radio enabled pin, if pulled down, power to the radio will be cutoff
- G0: 26, Interrupt pin
- SCK: 27, Serial clock
- MISO: 14, Serial data out
- MOSI: 12, Serial data in
- CS: 13, SPI Chip select
- RST: RST, Reset radio

## GPS
- TX: TX, Serial transmit
- RX: RX, Serial receive

## UI
2 buttons eventually connected to GPIO

## Microcontroller
Pins 13 and 0 must be set up as an SPI interface
Pin 18 must be set up as an I2C interface
Pins 19, 27, and 16 must be setup as clocks
Pins 5 and 26 must be setup for interrupts
