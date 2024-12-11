## Pinout descriptions

**Legend:** \<peripheral pin\> : \<esp32 gpio pin\>, \<description\>

## IMU
- `SCL`: `22`, Serial clock
- `SDA`: `19`, Serial in
- `INT`: `23`, Interrupt pin

## Screen
- `RST`: `15`, Reset screen
- `CS`: `0`, SPI chip select
- `DC`: `4`, Data or command
- `SDA`: `17`, Serial in
- `SCL`: `16`, Serial clock

## Lora
- `EN`: `25`, Radio enabled pin, if pulled down, power to the radio will be cutoff
- `G0`: `26`, Interrupt pin
- `SCK`: `27`, Serial clock
- `MISO`: `14`, Serial data out
- `MOSI`: `12`, Serial data in
- `CS`: `13`, SPI Chip select
- `RST`: `RST`, Reset radio

## GPS
- `TX`: `34`, GPS uart transmit
- `RX`: `33`, GPS uart receive

## Buttons
These must be configured as pull-up resistors in software, active low. We directly wire each GPIO to one side of the button switch, and the other is grounded.
- `button 1`: `32`, Pairing toggle
- `button 2`: `21`, Host/join toggle
