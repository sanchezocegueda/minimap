#include <nmea_parser.h>
#include "lora.h"


// SCREEN
// #include <screen.h>

// IMU
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

#include <ahrs.h>
#include <mpu9250.h>
#include <calibrate.h>
#include <common.h>
// IMU
#include "imu_irq.h"
#include "esp_system.h"
#include "mbedtls/aes.h" // need to include esp_system.h too??

extern QueueHandle_t imu_irq_queue;

static const char *IMU_TAG = "[IMU]";

calibration_t cal = {
    .mag_offset = {.x = 8.476562, .y = 11.578125, .z = 9.960938},
    .mag_scale = {.x = 0.970968, .y = 1.035335, .z = 0.995789},
    .gyro_bias_offset = {.x = -0.840523, .y = -0.754751, .z = -1.463743},
    .accel_offset = {.x = 0.019448, .y = 0.045232, .z = 0.084993},
    .accel_scale_lo = {.x = 1.021489, .y = 1.022823, .z = 1.049751},
    .accel_scale_hi = {.x = -0.980145, .y = -0.977748, .z = -0.968229}};


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

  v->x = -x;
  v->y = -z;
  v->z = -y;
}

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

  v->x = -y;
  v->y = z;
  v->z = -x;
}

void run_imu(void)
{
  vector_t va, vg, vm;
  // Get the Accelerometer, Gyroscope and Magnetometer values.
  ESP_ERROR_CHECK(get_accel_gyro_mag(&va, &vg, &vm));

  // Transform these values to the orientation of our device.
  transform_accel_gyro(&va);
  transform_accel_gyro(&vg);
  transform_mag(&vm);

  // Apply the AHRS algorithm
  ahrs_update(DEG2RAD(vg.x), DEG2RAD(vg.y), DEG2RAD(vg.z),
              va.x, va.y, va.z,
              vm.x, vm.y, vm.z);

  float temp;
  ESP_ERROR_CHECK(get_temperature_celsius(&temp));

  float heading, pitch, roll;
  ahrs_get_euler_in_degrees(&heading, &pitch, &roll);
  ESP_LOGI(IMU_TAG, "heading: %2.3f°, pitch: %2.3f°, roll: %2.3f°, Temp %2.3f°C", heading, pitch, roll, temp);
}

static void imu_task(void *arg)
{

#ifdef CONFIG_CALIBRATION_MODE
  calibrate_gyro();
  calibrate_accel();
  calibrate_mag();
#else
  i2c_mpu9250_init(&cal);
  imu_irq_init(); // Attach isr to INT pin, we will block on imu_irq_queue
  ahrs_init(SAMPLE_FREQ_Hz, 0.8);

  for (;;) {
    int event;
    if (xQueueReceive(imu_irq_queue, &event, portMAX_DELAY) && event == 1) {
      run_imu();
    }
    vTaskDelay(1000);
  }
#endif

  // Exit
  vTaskDelay(100 / portTICK_PERIOD_MS);
  i2c_driver_delete(I2C_MASTER_NUM);

  vTaskDelete(NULL);
}

// END IMU, BEGIN GPS 

/* NMEA Parser example, that decode data stream from GPS receiver

   This example code is in the Public Domain (or CC0 licensed, at your option.)

   Unless required by applicable law or agreed to in writing, this
   software is distributed on an "AS IS" BASIS, WITHOUT WARRANTIES OR
   CONDITIONS OF ANY KIND, either express or implied.
*/

#include <stdio.h>
#include <stdlib.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_log.h"
#include "nmea_parser.h"

static const char *GPS_TAG = "[GPS]";

#define TIME_ZONE (-8)   // PST
#define YEAR_BASE (2000) //date in GPS starts from 2000

/**
 * @brief GPS Event Handler
 *
 * @param event_handler_arg handler specific arguments
 * @param event_base event base, here is fixed to ESP_NMEA_EVENT
 * @param event_id event id
 * @param event_data event specific arguments
 */
static void gps_event_handler(void *event_handler_arg, esp_event_base_t event_base, int32_t event_id, void *event_data)
{
    gps_t *gps = NULL;
    switch (event_id) {
    case GPS_UPDATE:
        gps = (gps_t *)event_data;
        /* print information parsed from GPS statements */
        ESP_LOGI(GPS_TAG, "%d/%d/%d %d:%d:%d => \r\n"
                 "\t\t\t\t\t\tlatitude   = %.05f°N\r\n"
                 "\t\t\t\t\t\tlongitude = %.05f°E\r\n"
                 "\t\t\t\t\t\taltitude   = %.02fm\r\n"
                 "\t\t\t\t\t\tspeed      = %fm/s",
                 gps->date.year + YEAR_BASE, gps->date.month, gps->date.day,
                 gps->tim.hour + TIME_ZONE, gps->tim.minute, gps->tim.second,
                 gps->latitude, gps->longitude, gps->altitude, gps->speed);
        break;
    case GPS_UNKNOWN:
        /* print unknown statements */
        ESP_LOGW(GPS_TAG, "Unknown statement:%s", (char *)event_data);
        break;
    default:
        break;
    }
}


void app_main(void) {
  /* NMEA parser configuration */
  nmea_parser_config_t config = NMEA_PARSER_CONFIG_DEFAULT();
  /* init NMEA parser library */
  nmea_parser_handle_t nmea_hdl = nmea_parser_init(&config);
  /* register event handler for NMEA parser library */
  nmea_parser_add_handler(nmea_hdl, gps_event_handler, NULL);
  xTaskCreate(imu_task, "imu_task", 4096, NULL, 10, NULL);
}
