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

static const char *GPS_TAG = "[GPS]";

#define TIME_ZONE (-8)   // PST
#define YEAR_BASE (2000) //date in GPS starts from 2000

static void gps_event_handler(void *event_handler_arg, esp_event_base_t event_base, int32_t event_id, void *event_data);
static void send_lora_gps(float latitude, float longitude, float altitude);
// GPS

static uint32_t counter = 0;

void task_tx(void *p)
{
   ESP_LOGI("[TX]", "Starting");
   for(;;) {
      lora_send_packet((uint8_t*)&counter, 4);
      ESP_LOGI("[TX]", "counter: %ld", counter);
      counter++;
      vTaskDelay(pdMS_TO_TICKS(2500));
   }
}

uint8_t buf[4];

void task_rx(void *p)
{
   int x;
   for(;;) {
      x = lora_receive_packet(buf, sizeof(buf));
      ESP_LOGI("[RX]", "%d bytes, counter: %ld", x, (uint32_t) buf[0]);
      vTaskDelay(1);
   }
}

void app_main()
{
   lora_init();
   lora_set_frequency(915e6);
   lora_enable_crc();

//   /* NMEA parser configuration */
//   nmea_parser_config_t config = NMEA_PARSER_CONFIG_DEFAULT();
//   /* init NMEA parser library */
//   nmea_parser_handle_t nmea_hdl = nmea_parser_init(&config);
//   /* register event handler for NMEA parser library */
//   nmea_parser_add_handler(nmea_hdl, gps_event_handler, NULL);

   // xTaskCreate(&task_tx, "task_tx", 2048, NULL, 5, NULL);
   xTaskCreate(&task_rx, "task_rx", 2048, NULL, 5, NULL);
}


/**
 * @brief GPS Event Handler
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
   const char* format =  "Lat: %0.3f\nLong: %0.3f\nAlt: %0.3f\n";
   uint32_t format_len = 26;
   ESP_LOGW(GPS_TAG, "sizeof format %ld", format_len);
   uint32_t msg_size = 3 * sizeof(float);
   char buf[msg_size + format_len];
   snprintf(buf, msg_size + format_len, format, latitude, longitude, altitude);
   ESP_LOG_BUFFER_CHAR(GPS_TAG, buf, msg_size + format_len);
   lora_send_packet((uint8_t*)buf, msg_size + format_len);
}