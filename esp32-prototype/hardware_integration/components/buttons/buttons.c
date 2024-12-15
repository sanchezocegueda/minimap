#include "buttons.h"

/* You could also pass data into USR_DATA with iot_button_register_cb */
void left_cb(void* arg, void *usr_data)
{
    minimap_button_event_t val = LEFT_PRESS;
    xQueueSendToFront(button_event_queue, &val, portMAX_DELAY);
}


void right_cb(void* arg, void *usr_data)
{
    minimap_button_event_t val = RIGHT_PRESS;
    xQueueSendToFront(button_event_queue, &val, portMAX_DELAY);
}


/* Task to handle button interrupts. This should be modified each time the button callbacks change.
(or deleted and a new listener spawned...) */
void button_listener(void* arg)
{
    uint32_t event;
    for (;;) {
        if (xQueueReceive(button_event_queue, &event, portMAX_DELAY)) {
            if (event == LEFT_PRESS) {
                ESP_LOGI("[BUTTON]", "Left button pressed!");
            } else if (event == RIGHT_PRESS) {
                ESP_LOGI("[BUTTON]", "Right button pressed!");
            }
        }
    }
}

/* Create an IDF, GPIO button attached to GPIO_PIN. */
button_handle_t init_btn(gpio_num_t gpio_pin) {
    button_config_t btn_cfg = {
        .type = BUTTON_TYPE_GPIO,
        .long_press_time = CONFIG_BUTTON_LONG_PRESS_TIME_MS,
        .short_press_time = CONFIG_BUTTON_SHORT_PRESS_TIME_MS,
        .gpio_button_config = {
            .gpio_num = gpio_pin,
            .active_level = 0,
        },
    };
    button_handle_t btn = iot_button_create(&btn_cfg);
    if (btn == NULL) {
        ESP_LOGE("ERROR", "Button failed to create");
    }
    return btn;
}

/* TODO: Not quite sure where we want the queue. For now it's defined in main.c
and that address is passed to this function. */

/* Setup GPIO for the buttons and creates a queue and a listener task for button events.
In order to use the buttons, register callback functions with iot_button_register_cb. */
void init_buttons(button_handle_t *left, button_handle_t *right)
{
    *left = init_btn(LEFT_BUTTON_PIN);
    *right = init_btn(RIGHT_BUTTON_PIN);
    button_event_queue = xQueueCreate(16, sizeof(minimap_button_event_t));
    xTaskCreate(button_listener, "button listener", 2048, NULL, 10, NULL);
    ESP_LOGI("BUTTON INIT DONE", "");
}