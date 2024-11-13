when using the IMU, make sure GPIO 2 is grounded!
- still not sure why this is

# Interrupts
<https://docs.espressif.com/projects/esp-idf/en/stable/esp32/api-reference/system/intr_alloc.html#iram-safe-interrupt-handlers>

<https://esp32tutorials.com/esp32-gpio-interrupts-esp-idf/>

<https://docs.espressif.com/projects/esp-idf/en/stable/esp32/api-reference/system/intr_alloc.html>

We need to pin tasks that interrupt to cores, use `xTaskCreatePinnedToCore()`
- <https://docs.espressif.com/projects/esp-idf/en/stable/esp32/api-reference/system/intr_alloc.html#external-peripheral-interrupts>
- Use IRAM-Safe interrupts <https://docs.espressif.com/projects/esp-idf/en/stable/esp32/api-reference/system/intr_alloc.html#iram-safe-interrupt-handlers>

