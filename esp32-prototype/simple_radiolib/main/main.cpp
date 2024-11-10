/*
   RadioLib Non-Arduino ESP-IDF Example

   This example shows how to use RadioLib without Arduino.
   In this case, a Liligo T-BEAM (ESP32 and SX1276)
   is used.

   Can be used as a starting point to port RadioLib to any platform!
   See this API reference page for details on the RadioLib hardware abstraction
   https://jgromes.github.io/RadioLib/class_hal.html

   For full API reference, see the GitHub Pages
   https://jgromes.github.io/RadioLib/
*/

// include the library
#include <RadioLib.h>

// include the hardware abstraction layer
#include "hal/ESP-IDF/EspHal.h"

// Luca: SPI configured for radio
// Will want to define the pins in a header...

// Luca: Configure HAL for RadioLib API
EspHal* hal = new EspHal(27, 14, 12);

// Luca: I believe the additional gpio is where we want the Lora EN pin connected to
RFM95 radio = new Module(hal, 13, 26, RADIOLIB_NC, 25);

static const char *TAG = "main";

// the entry point for the program
// it must be declared as "extern C" because the compiler assumes this will be a C function
extern "C" void app_main(void) {
  // initialize just like with Arduino
  ESP_LOGI(TAG, "[RFM95] Initializing ... ");
  int state = radio.begin();
  if (state != RADIOLIB_ERR_NONE) {
    ESP_LOGI(TAG, "failed, code %d\n", state);
    while(true) {
      hal->delay(1000);
    }
  }
  ESP_LOGI(TAG, "success!\n");

  // loop forever
  for(;;) {
    // send a packet
    ESP_LOGI(TAG, "[SX1276] Transmitting packet ... ");
    state = radio.transmit("Hello World!");
    if(state == RADIOLIB_ERR_NONE) {
      // the packet was successfully transmitted
      ESP_LOGI(TAG, "success!");

    } else {
      ESP_LOGI(TAG, "failed, code %d\n", state);

    }

    // wait for a second before transmitting again
    hal->delay(1000);

  }

}