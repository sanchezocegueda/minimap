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

static const char *GPS_TAG = "[GPS]";

#define TIME_ZONE (-8)   // PST
#define YEAR_BASE (2000) //date in GPS starts from 2000

static void gps_event_handler(void *event_handler_arg, esp_event_base_t event_base, int32_t event_id, void *event_data);
static void send_lora_gps(float latitude, float longitude, float altitude);
// GPS


/* Basic transmit task to send a counter every 2.5 seconds */
void task_tx(void *p)
{
   static uint32_t counter = 0;
   ESP_LOGI("[TX]", "Starting");
   for(;;) {
      lora_send_packet((uint8_t*)&counter, 4);
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
      x = lora_receive_packet(buf, sizeof(buf));
      ESP_LOGI("[RX]", "%d bytes, counter: %ld", x, (uint32_t) buf[0]);
      vTaskDelay(1);
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
    gps_t *gps = NULL;
    switch (event_id) {
    case GPS_UPDATE:
        gps = (gps_t *)event_data;
        /* print information parsed from GPS statements */
        ESP_LOGI(GPS_TAG, "%d/%d/%d %d:%d:%d => \r\n"
                 "\t\t\t\t\t\tlatitude   = %.05f°N\r\n"
                 "\t\t\t\t\t\tlongitude = %.05f°E\r\n"
                 "\t\t\t\t\t\taltitude   = %.02fm\r\n"
                 "\t\t\t\t\t\tspeed      = %fm/s",
                 gps->date.year + YEAR_BASE, gps->date.month, gps->date.day,
                 gps->tim.hour + TIME_ZONE, gps->tim.minute, gps->tim.second,
                 gps->latitude, gps->longitude, gps->altitude, gps->speed);
         send_lora_gps(gps->latitude, gps->longitude, gps->altitude);
        break;
    case GPS_UNKNOWN:
        /* print unknown statements */
        ESP_LOGW(GPS_TAG, "Unknown statement:%s", (char *)event_data);
        break;
    default:
        break;
    }
}


/* Format latitude, longitude, altitude and send via lora */
static void send_lora_gps(float latitude, float longitude, float altitude) {
   float buf[3] = {latitude, longitude, altitude};
   lora_send_packet((uint8_t*)buf, 3 * sizeof(float));
   ESP_LOGI("[LORA_TX_GPS]", "Lat: %0.5f, Long: %0.5f, Altitude: %0.5f", latitude, longitude, altitude);
}

/* Task for receiving GPS over lora */
void receive_lora_gps(void*) {
   float buf[3];
   for (;;) {
      lora_receive_packet((uint8_t*)buf, 3 * sizeof(float));
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

   // Setup GPS event handler to send data using `send_lora_gps`
//   nmea_parser_config_t config = NMEA_PARSER_CONFIG_DEFAULT();
//   nmea_parser_handle_t nmea_hdl = nmea_parser_init(&config);
//   nmea_parser_add_handler(nmea_hdl, gps_event_handler, NULL);

   // xTaskCreate(&task_tx, "task_tx", 2048, NULL, 5, NULL);
   xTaskCreate(&task_rx, "task_rx", 2048, NULL, 5, NULL);
   // xTaskCreate(&receive_lora_gps, "task_lora_rx", 2048, NULL, 5, NULL);
}

