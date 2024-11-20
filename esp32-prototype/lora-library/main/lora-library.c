#include <stdio.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "lora.h"
#include "esp_log.h"

static uint32_t counter = 0;

void task_tx(void *p)
{
   for(;;) {
      lora_send_packet((uint8_t*)&counter, 4);
      ESP_LOGI("[SENT]", "counter: %ld", counter);
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
      ESP_LOGI("[RECEIVED]", "%d bytes, counter: %ld", x, (uint32_t) buf[0]);
      vTaskDelay(1);
   }
}

void app_main()
{
   lora_init();
   lora_set_frequency(915e6);
   lora_enable_crc();
   // xTaskCreate(&task_tx, "task_tx", 2048, NULL, 5, NULL);
   xTaskCreate(&task_rx, "task_rx", 2048, NULL, 5, NULL);
}