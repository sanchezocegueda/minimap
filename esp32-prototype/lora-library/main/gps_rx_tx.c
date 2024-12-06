#include <stdio.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "lora.h"
#include "esp_log.h"
#include "string.h"

// GPS
#include <stdlib.h>
#include "nmea_parser.h"
// #include <stdio.h>
// #include "freertos/FreeRTOS.h"
// #include "freertos/task.h"
// #include "esp_log.h"

// shouldn't need 
// #include "esp_system.h"
#include "mbedtls/aes.h"


static void gps_event_handler(void *event_handler_arg, esp_event_base_t event_base, int32_t event_id, void *event_data);
static void send_lora_gps(float latitude, float longitude, float altitude);
// GPS


/* Basic transmit task to send a counter every 2.5 seconds */
void task_tx(void *p)
{
   static uint32_t counter = 0;
   ESP_LOGI("[TX]", "Starting");
   for(;;) {
      lora_send_packet_blocking((uint8_t*)&counter, 4);
      ESP_LOGI("[TX]", "counter: %ld", counter);
      counter++;
      vTaskDelay(pdMS_TO_TICKS(2500));
   }
}


/* Basic receive task to turn radio into receive mode and block until data is received */
void task_rx(void *p)
{
   uint8_t buf[4];
   int x;
   for(;;) {
      x = lora_receive_packet_blocking(buf, sizeof(buf));
      ESP_LOGI("[RX]", "%d bytes, counter: %ld", x, (uint32_t) buf[0]);
      vTaskDelay(1);
   }
}

/* Task to send a counter and listen for a counter periodically*/
void task_both(void *p) {
   
   for(;;) {

   }
}

/**
 * @brief GPS Event Handler, logs the GPS data to serial and transmits latitude, longitude, and altitude over lora.
 *
 * @param event_handler_arg handler specific arguments
 * @param event_base event base, here is fixed to ESP_NMEA_EVENT
 * @param event_id event id
 * @param event_data event specific arguments
 */
static void gps_event_handler(void *event_handler_arg, esp_event_base_t event_base, int32_t event_id, void *event_data)
{
   gps_t *gps = (gps_t *)event_data;
   gps_debug(gps);
   send_lora_gps(gps->latitude, gps->longitude, gps->altitude);
}

/* Format latitude, longitude, altitude and send via lora */
static void send_lora_gps(float latitude, float longitude, float altitude) {
   // encryption 
   float buf[3] = {latitude, longitude, altitude};
   
   lora_send_packet_blocking((uint8_t*)buf, 3 * sizeof(float));
   ESP_LOGI("[LORA_TX_GPS]", "Lat: %0.5f, Long: %0.5f, Altitude: %0.5f", latitude, longitude, altitude);
}

/* Task for receiving GPS over lora */
void receive_lora_gps(void*) {
   float buf[3];
   for (;;) {
      lora_receive_packet_blocking((uint8_t*)buf, 3 * sizeof(float));
      ESP_LOGI("[LORA_RX_GPS]", "Lat: %0.5f, Long: %0.5f, Altitude: %0.5f", buf[0], buf[1], buf[2]);
      vTaskDelay(1);
   }
}

/* Uncomment only the NMEA stuff to load a GPS transmit binary, uncomment only one of a given rx counter task, tx counter task, or receive_lora_gps task */
void app_main()
{
   lora_init();
   lora_set_frequency(915e6);
   lora_enable_crc();

   // Varun Encryption stuff ===============
   // char key[256];  // defaulting to zeros
   // unsigned int keybits = 256;

   // mbedtls_aes_xts_context ctx;
   // mbedtls_aes_xts_context * ctx_ptr = &ctx;
   // mbedtls_aes_xts_init(ctx_ptr);

   // mbedtls_aes_xts_setkey_enc(ctx_ptr, key, keybits)
   // Varun Encryption stuff ===============


   // Setup GPS event handler to send data using `send_lora_gps`
//   nmea_parser_config_t config = NMEA_PARSER_CONFIG_DEFAULT();
//   nmea_parser_handle_t nmea_hdl = nmea_parser_init(&config);
//   nmea_parser_add_handler(nmea_hdl, gps_event_handler, NULL, SCREEN_UPDATE);

   // xTaskCreate(&task_tx, "task_tx", 2048, NULL, 5, NULL);
   xTaskCreate(&task_rx, "task_rx", 2048, NULL, 5, NULL);
   // xTaskCreate(&receive_lora_gps, "task_lora_rx", 2048, NULL, 5, NULL);
}

