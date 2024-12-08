#include <stdio.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/queue.h"
#include "driver/gpio.h"
#include "esp_err.h"
#include "esp_log.h"
#include "iot_button.h"

#define LEFT_BUTTON_PIN 32
#define RIGHT_BUTTON_PIN 21

extern QueueHandle_t button_event_queue;

void left_cb(void* arg, void *usr_data);

void right_cb(void* arg, void *usr_data);

/* Task to handle button interrupts. */
void button_listener(void* arg);

/* Create an IDF, GPIO button attached to GPIO_PIN. */
button_handle_t init_btn(gpio_num_t gpio_pin);

void init_buttons(button_handle_t *left, button_handle_t *right, QueueHandle_t *button_event_queue);