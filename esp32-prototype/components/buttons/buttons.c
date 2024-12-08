#include "buttons.h"

void button_single_click_cb(void* arg, void *usr_data)
{
    uint32_t gpio_num = LEFT_BUTTON_PIN;
    xQueueSendToFront(button_event_queue, &gpio_num, portMAX_DELAY);
}

/* Task to handle button interrupts. */
void button_listener(void* arg)
{
    uint32_t io_num;
    for (;;) {
        if (xQueueReceive(button_event_queue, &io_num, portMAX_DELAY)) {
            ESP_LOGI("[BUTTON]", "Interrupt received!");
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
