# register the component and set "RadioLib", "esp_timer" and "driver" as required for main.cpp to run
idf_component_register(SRCS "main.cpp"
                       REQUIRES RadioLib esp_timer driver)