#include <stdio.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "lora.h"
#include "esp_log.h"

static uint32_t counter = 0;

void task_tx(void *p)
{
   for(;;) {
      lora_send_packet((uint8_t*)&counter, 7);
      ESP_LOGI("[SENT]", "counter: %ld", counter);
      vTaskDelay(pdMS_TO_TICKS(2500));
      counter++;
   }
}

uint8_t buf[32];

void task_rx(void *p)
{
   int x;
   for(;;) {
      lora_receive();    // put into receive mode
      while(lora_received()) {
         x = lora_receive_packet(buf, sizeof(buf));
         // buf[x] = 0;
         // printf("Received: %s\n", buf);

         ESP_LOGI("[RECEIVED]", "%d bytes, counter: %ld", x, (uint32_t) buf[0]);
         lora_receive();
      }
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