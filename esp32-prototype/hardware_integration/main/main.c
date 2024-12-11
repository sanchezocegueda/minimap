#include <stdio.h>
#include <string.h>
#include <stdlib.h>

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/queue.h"

#include "esp_log.h"
#include "esp_system.h"
#include "esp_err.h"
#include "esp_task_wdt.h"

#include "driver/i2c.h"

#include "nmea_parser.h"
#include "screen.h"

/* Code for our 2 physical buttons */
#include "buttons.h"

/* IMU */
#include "ahrs.h"
#include "mpu9250.h"
#include "calibrate.h"
#include "common.h"

#include "lora.h"

#include "psa/crypto.h"

/* TODO: Cleanup */
void send_lora_gps();

#define I2C_MASTER_NUM I2C_NUM_0 /* I2C port number for master dev */

/* GPS Coordinate */
typedef struct coordinates
{
    float lat;
    float lon;
} coordinates_t;

coordinates_t other;

/* Global variable init */
imu_data_t global_imu;
gps_t global_gps;
QueueHandle_t button_event_queue;

/* IMU Calibration, set accordingly in run_imu */
calibration_t cal_mpu92_65 = {
    .mag_offset = {.x = 46.160156, .y = -11.281250, .z = 21.173828},
    .mag_scale = {.x = 1.013871, .y = 0.983708, .z = 1.002889},
    .gyro_bias_offset = {.x = -2.262275, .y = -1.421279, .z = -2.291301},
    .accel_offset = {.x = 0.009100, .y = 0.053621, .z = 0.056311},
    .accel_scale_lo = {.x = 0.999614, .y = 1.015274, .z = 1.023689},
    .accel_scale_hi = {.x = -0.997792, .y = -0.985116, .z = -1.002624}};

calibration_t cal_mpu9250_6500 = {
    .mag_offset = {.x = 47.226562, .y = 17.671875, .z = 11.132812},
    .mag_scale = {.x = 0.943459, .y = 1.238415, .z = 0.882935},
    .gyro_bias_offset = {.x = -1.029911, .y = -0.749238, .z = -1.779955},
    .accel_offset = {.x = 0.027026, .y = 0.049613, .z = 0.035468},
    .accel_scale_lo = {.x = 1.014326, .y = 1.026735, .z = 1.031887},
    .accel_scale_hi = {.x = -0.986112, .y = -0.977564, .z = -0.994764}};


/**
 * Transformation:
 *  - Rotate around Z axis 180 degrees
 *  - Rotate around X axis -90 degrees
 * @param  {object} s {x,y,z} sensor
 * @return {object}   {x,y,z} transformed
 */
static void transform_accel_gyro(vector_t *v)
{
  float x = v->x;
  float y = v->y;
  float z = v->z;

  v->x = y;
  v->y = x;
  v->z = -z;
}


// TODO: technically not needed if we aren't changing the mag axes
/**
 * Transformation: to get magnetometer aligned
 * @param  {object} s {x,y,z} sensor
 * @return {object}   {x,y,z} transformed
 */
static void transform_mag(vector_t *v)
{
  float x = v->x;
  float y = v->y;
  float z = v->z;

  v->x = x;
  v->y = y;
  v->z = z;
}


void run_imu(void)
{
  i2c_mpu9250_init(&cal_mpu9250_6500);
  ahrs_init(SAMPLE_FREQ_Hz, 0.8);

  uint64_t i = 0;
  while (true)
  {
    vector_t va, vg, vm;

    /* Get the raw Accelerometer, Gyroscope and Magnetometer values. */
    ESP_ERROR_CHECK(get_accel_gyro_mag(&va, &vg, &vm));

    /* Transform these to the orientation of our device. */
    transform_accel_gyro(&va);
    transform_accel_gyro(&vg);
    transform_mag(&vm);

    /* Apply the AHRS algorithm */
    ahrs_update(DEG2RAD(vg.x), DEG2RAD(vg.y), DEG2RAD(vg.z),
                va.x, va.y, va.z,
                vm.x, vm.y, vm.z);

    /* Update global_imu data */
    float heading, pitch, roll;
    ahrs_get_euler_in_degrees(&heading, &pitch, &roll);
    global_imu.heading = heading;
    global_imu.pitch = pitch;
    global_imu.roll = roll;

    if (i++ % 100 == 0)
    {
      // float temp;
      // ESP_ERROR_CHECK(get_temperature_celsius(&temp));

      ESP_LOGI("[IMU]", "heading: %2.3f°, pitch: %2.3f°, roll: %2.3f°", heading, pitch, roll);

      // Make the WDT happy
      // vTaskDelay(1);
    }
    // no idea why this is here
    imu_pause();
  }
}


static void imu_task(void *arg)
{
#ifdef CONFIG_CALIBRATION_MODE
  calibrate_gyro();
  calibrate_accel();
  calibrate_mag();
#else
  run_imu();
#endif

  // Exit
  vTaskDelay(100 / portTICK_PERIOD_MS);
  i2c_driver_delete(I2C_MASTER_NUM);

  vTaskDelete(NULL);
}


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
      lora_receive();
      while(lora_received()) {
        x = lora_receive_packet(buf, sizeof(buf));
        ESP_LOGI("[RX]", "%d bytes, counter: %ld", x, (uint32_t) buf[0]);
      }
      vTaskDelay(1);
   }

}

/* Task to send a counter and listen for a counter periodically*/
void task_both(void *p)
{
  ESP_LOGI("[task both]", "started");
  static uint32_t counter = 0;
  uint8_t buf[4];
  int bytes_read;
  for (;;)
  {
    lora_send_packet_blocking((uint8_t *)&counter, 4);
    ESP_LOGI("[TX]", "counter: %ld", counter);
    bytes_read = lora_receive_packet_blocking(buf, sizeof(buf));

    ESP_LOGI("[RX]", "%d bytes, counter: %ld", bytes_read, *((uint32_t*)buf));
    counter++;
    vTaskDelay(1);
  }
}

/* Task to send a counter and listen for a counter periodically*/
void task_both_gps(void *p)
{
  ESP_LOGI("[task both gps]", "started");
  float buf[3];
  int bytes_read;
  for (;;)
  {
    send_lora_gps();
    int bytes_read = lora_receive_packet_blocking((uint8_t*)buf, 3 * sizeof(float));
    ESP_LOGI("[LORA_RX_GPS]", "Bytes read: %d, Lat: %0.5f, Long: %0.5f, Altitude: %0.5f", bytes_read, buf[0], buf[1], buf[2]);
    other.lat = buf[0];
    other.lon = buf[1];
    vTaskDelay(1);
  }
}

/* Update global_gps, event handler for nmea_event. TODO: switch to thread-safe ds */
void update_global_gps(void *event_handler_arg, esp_event_base_t event_base, int32_t event_id, void *event_data)
{
   global_gps = *((gps_t *)event_data);
  //  gps_debug(&global_gps);
  //  send_lora_gps();
}


/* Format latitude, longitude, altitude and send via lora */
void send_lora_gps() {
  float latitude = global_gps.latitude;
  float longitude = global_gps.longitude;
  float altitude = global_gps.altitude;

   // TODO: encryption 
   float buf[3] = {latitude, longitude, altitude};
   
   lora_send_packet_blocking((uint8_t*)buf, 3 * sizeof(float));
   ESP_LOGI("[LORA_TX_GPS]", "Lat: %0.5f, Long: %0.5f, Altitude: %0.5f", latitude, longitude, altitude);
}


/* Task for receiving GPS over lora */
void receive_lora_gps(void*) {
   float buf[3];
   for (;;) {
      lora_receive();
      while(lora_received()) {
        int bytes_read = lora_receive_packet((uint8_t*)buf, 3 * sizeof(float));
        ESP_LOGI("[LORA_RX_GPS]", "Bytes read: %d, Lat: %0.5f, Long: %0.5f, Altitude: %0.5f", bytes_read, buf[0], buf[1], buf[2]);
      }
      vTaskDelay(1);
   }
}

/* Testing encryption */

void app_main()
{
  /* Setup GPS, TODO: change event loop stuff */
  nmea_parser_config_t config = NMEA_PARSER_CONFIG_DEFAULT();
  nmea_parser_handle_t nmea_hdl = nmea_parser_init(&config);
  
  /* Setup IMU and start the polling task. */
  xTaskCreate(imu_task, "imu_task", 4096, NULL, 10, NULL);

  /* Setup buttons */
  button_handle_t left_btn, right_btn;
  init_buttons(&left_btn, &right_btn, &button_event_queue);

  iot_button_register_cb(left_btn, BUTTON_PRESS_DOWN, left_cb, NULL);
  iot_button_register_cb(right_btn, BUTTON_PRESS_DOWN, right_cb, NULL);

  /* Setup Lora Radio */
  lora_init();
  lora_set_frequency(915e6);
  lora_enable_crc();

  nmea_parser_add_handler(nmea_hdl, update_global_gps, NULL, SCREEN_UPDATE);

  // xTaskCreate(&task_tx, "task_tx", 4096, NULL, 5, NULL);
  // xTaskCreate(&task_rx, "task_rx", 4096, NULL, 5, NULL);
  xTaskCreate(&task_both_gps, "task_both", 4096, NULL, 5, NULL);
  // xTaskCreate(&receive_lora_gps, "task_lora_rx", 2048, NULL, 5, NULL);

  // test_encryption();
  xTaskCreate(screen_main_task, "Minimap", LVGL_TASK_STACK_SIZE, NULL, LVGL_TASK_PRIORITY, NULL);
}
