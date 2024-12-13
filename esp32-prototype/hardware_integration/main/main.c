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

coordinates_t other;

/* Global variable init */
imu_data_t global_imu;
nmea_parser_handle_t nmea_hndl;
QueueHandle_t button_event_queue;

/* IMU Calibration, set accordingly in run_imu */
calibration_t cal_mpu92_65 = {
    .mag_offset = {.x = 44.384766, .y = -12.468750, .z = 24.607422},
    .mag_scale = {.x = 1.023329, .y = 1.019963, .z = 0.959353},
    .gyro_bias_offset = {.x = -2.226222, .y = -1.335986, .z = -2.275180},
    .accel_offset = {.x = -0.010997, .y = 0.026934, .z = 0.042839},
    .accel_scale_lo = {.x = 1.000212, .y = 1.015479, .z = 1.029726},
    .accel_scale_hi = {.x = -1.001975, .y = -0.988439, .z = -0.996091}};

calibration_t cal_mpu9250_6500 = {
    .mag_offset = {.x = 9.082031, .y = 21.937500, .z = -2.929688},
    .mag_scale = {.x = 0.992484, .y = 1.201046, .z = 0.862203},
    .gyro_bias_offset = {.x = -0.612455, .y = -0.792115, .z = -1.854843},
    .accel_offset = {.x = 0.028677, .y = 0.059277, .z = 0.060068},
    .accel_scale_lo = {.x = 1.016573, .y = 1.025387, .z = 1.041263},
    .accel_scale_hi = {.x = -0.984251, .y = -0.977627, .z = -0.985206}};




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

    if (i++ % 10 == 0)
    {
      // float temp;
      // ESP_ERROR_CHECK(get_temperature_celsius(&temp));

      ESP_LOGI("[IMU]", "heading: %2.3f°, pitch: %2.3f°, roll: %2.3f°", heading, pitch, roll);

      // Make the WDT happy
      vTaskDelay(1);
    }
    // no idea why this is here
    // imu_pause();
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
  coordinates_t buf;
  int bytes_read;
  for (;;)
  {
    send_lora_gps(nmea_hndl);
    int bytes_read = lora_receive_packet_blocking((uint8_t*)&buf, sizeof(coordinates_t));
    ESP_LOGI("[LORA_RX_GPS]", "Bytes read: %d, Lat: %0.5f, Long: %0.5f", bytes_read, buf.lat, buf.lon);
    other = buf;
    vTaskDelay(1);
  }
}


/**
 * @brief Format latitude, longitude and send via lora
 * @param nmea_hndl returned from nmea_parser_init
 * */
void send_lora_gps(nmea_parser_handle_t nmea_hndl) {
  coordinates_t buf = read_gps(nmea_hndl);
   // TODO: encryption 
   
   lora_send_packet_blocking((uint8_t*)&buf, sizeof(coordinates_t));
   ESP_LOGI("[LORA_TX_GPS]", "Lat: %0.5f, Long: %0.5f", buf.lat, buf.lon);
}


/* Task for receiving GPS over lora */
void receive_lora_gps(void*) {
   coordinates_t buf;
   for (;;) {
      lora_receive();
      while(lora_received()) {
        int bytes_read = lora_receive_packet((uint8_t*)&buf, sizeof(coordinates_t));
        ESP_LOGI("[LORA_RX_GPS]", "Bytes read: %d, Lat: %0.5f, Long: %0.5f", bytes_read, buf.lat, buf.lon);
      }
      vTaskDelay(1);
   }
}

/* Testing encryption */

void app_main()
{
  /* Setup GPS */
  // nmea_parser_config_t config = NMEA_PARSER_CONFIG_DEFAULT();
  // nmea_hndl = nmea_parser_init(&config);

  /* Setup IMU and start the polling task. */
  // xTaskCreate(imu_task, "imu_task", 4096, NULL, 10, NULL);

  /* Setup buttons */
  button_handle_t left_btn, right_btn;
  init_buttons(&left_btn, &right_btn, &button_event_queue);

  iot_button_register_cb(left_btn, BUTTON_PRESS_DOWN, left_cb, NULL);
  iot_button_register_cb(right_btn, BUTTON_PRESS_DOWN, right_cb, NULL);

  /* Setup Lora Radio */
  lora_init();
  /* Move to a configuration function in lora.c along with the end of lora_init */
  lora_set_frequency(915e6);
  lora_enable_crc();
  /* Set spreading factor, bandwidth, sync word, implicit header mode (and all that follows)*/

  // xTaskCreate(&task_tx, "task_tx", 4096, NULL, 5, NULL);
  xTaskCreate(&task_rx, "task_rx", 4096, NULL, 5, NULL);
  // xTaskCreate(&task_both_gps, "task_both", 4096, NULL, 5, NULL);
  // xTaskCreate(&receive_lora_gps, "task_lora_rx", 2048, NULL, 5, NULL);

  // test_encryption();
  // screen_task_params_t *screen_params = malloc(sizeof(screen_task_params_t));
  // screen_params->global_imu = &global_imu;
  // screen_params->nmea_hndl = nmea_hndl;

  // xTaskCreate(screen_main_task, "Minimap", LVGL_TASK_STACK_SIZE, screen_params, LVGL_TASK_PRIORITY, NULL);

}
