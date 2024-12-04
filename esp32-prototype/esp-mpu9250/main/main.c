/*****************************************************************************
 *                                                                           *
 *  Copyright 2018 Simon M. Werner                                           *
 *                                                                           *
 *  Licensed under the Apache License, Version 2.0 (the "License");          *
 *  you may not use this file except in compliance with the License.         *
 *  You may obtain a copy of the License at                                  *
 *                                                                           *
 *      http://www.apache.org/licenses/LICENSE-2.0                           *
 *                                                                           *
 *  Unless required by applicable law or agreed to in writing, software      *
 *  distributed under the License is distributed on an "AS IS" BASIS,        *
 *  WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied. *
 *  See the License for the specific language governing permissions and      *
 *  limitations under the License.                                           *
 *                                                                           *
 *****************************************************************************/

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

#include "ahrs.h"
#include "mpu9250.h"
#include "calibrate.h"
#include "common.h"

#include "esp_log.h"

static const char *TAG = "main";

#define I2C_MASTER_NUM I2C_NUM_0 /*!< I2C port number for master dev */

calibration_t cal_mpu92_65 = {
    .mag_offset = {.x = 40.242188, .y = -38.000000, .z = -17.740234},
    .mag_scale = {.x = 1.013800, .y = 1.010465, .z = 0.976592},
    .gyro_bias_offset = {.x = -2.274468, .y = -1.300148, .z = -2.120367},
    .accel_offset = {.x = -0.007656, .y = 0.117932, .z = 0.021797},
    .accel_scale_lo = {.x = 0.993931, .y = 1.012075, .z = 1.023260},
    .accel_scale_hi = {.x = -0.998414, .y = -0.985158, .z = -0.995281}};

calibration_t cal_mpu9250_6500 = {
    .mag_offset = {.x = 26.035156, .y = 22.546875, .z = -16.992188},
    .mag_scale = {.x = 0.999656, .y = 1.020460, .z = 0.980675},
    .gyro_bias_offset = {.x = -0.702319, .y = -0.755465, .z = -1.832088},
    .accel_offset = {.x = 0.029607, .y = 0.114505, .z = 0.050498},
    .accel_scale_lo = {.x = 1.012515, .y = 1.022693, .z = 1.037164},
    .accel_scale_hi = {.x = -0.977422, .y = -0.970941, .z = -0.982524}};
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

    // Get the Accelerometer, Gyroscope and Magnetometer values.
    ESP_ERROR_CHECK(get_accel_gyro_mag(&va, &vg, &vm));

    // Transform these values to the orientation of our device.
    transform_accel_gyro(&va);
    transform_accel_gyro(&vg);
    transform_mag(&vm);

    if (i % 10 == 0) {
      ESP_LOGI("RAW DATA", "mag x: %f mag y: %f mag z: %f", vm.x, vm.y, vm.z);
    }

    // Apply the AHRS algorithm
    ahrs_update(DEG2RAD(vg.x), DEG2RAD(vg.y), DEG2RAD(vg.z),
                va.x, va.y, va.z,
                vm.x, vm.y, vm.z);

    // Print the data out every 10 items
    if (i++ % 10 == 0)
    {
      float temp;
      ESP_ERROR_CHECK(get_temperature_celsius(&temp));

      float heading, pitch, roll;
      ahrs_get_euler_in_degrees(&heading, &pitch, &roll);
      ESP_LOGI(TAG, "heading: %2.3f°, pitch: %2.3f°, roll: %2.3f°, Temp %2.3f°C", heading, pitch, roll, temp);

      // Make the WDT happy
      vTaskDelay(0);
    }

    pause();
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

void app_main(void)
{
  // start i2c task
  xTaskCreate(imu_task, "imu_task", 4096, NULL, 10, NULL);
}
