#include <stdio.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "lora.h"

void task_tx(void *p)
{
   for(;;) {
      vTaskDelay(pdMS_TO_TICKS(5000));
      lora_send_packet((uint8_t*)"Hello", 5);
      printf("packet sent...\n");
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
         buf[x] = 0;
         printf("Received: %s\n", buf);
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
//    xTaskCreate(&task_tx, "task_tx", 2048, NULL, 5, NULL);
   xTaskCreate(&task_rx, "task_rx", 2048, NULL, 5, NULL);
}