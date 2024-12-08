#include <stdio.h>
#include <stdlib.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_log.h"
// #include "esp_err.h"

#include "screen.h"
#include "nmea_parser.h"

#include "mpu9250.h"
#include "ahrs.h"
#include "common.h"
#include "calibrate.h"


imu_data_t global_imu;
gps_t global_gps;

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
    global_gps = *((gps_t *)event_data);
    global_gps.latitude += 1.1;
    // gps_debug(&global_gps);
    
    /* print information parsed from GPS statements */
    // gps_debug(&global_gps);
}

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

    // Apply the AHRS algorithm
    ahrs_update(DEG2RAD(vg.x), DEG2RAD(vg.y), DEG2RAD(vg.z),
                va.x, va.y, va.z,
                vm.x, vm.y, vm.z);


    float heading, pitch, roll;
    ahrs_get_euler_in_degrees(&heading, &pitch, &roll);
    global_imu.heading = heading;
    global_imu.pitch = pitch;
    global_imu.roll = roll;
    // Print the data out every 10 items
    if (i++ % 10 == 0)
    {
    //   float temp;
    //   ESP_ERROR_CHECK(get_temperature_celsius(&temp));
    //   ESP_LOGI("[IMU]", "heading: %2.3f°, pitch: %2.3f°, roll: %2.3f°", heading, pitch, roll);

      // Make the WDT happy
      vTaskDelay(0);
    }

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


void app_main(void)
{
    xTaskCreate(imu_task, "imu_task", 4096, NULL, 10, NULL);

    start_screen();
    /* NMEA parser configuration */
    // nmea_parser_config_t config = NMEA_PARSER_CONFIG_DEFAULT();
    /* init NMEA parser library */
    // nmea_parser_handle_t nmea_hdl = nmea_parser_init(&config);
    /* register event handler for NMEA parser library */
    // nmea_parser_add_handler(nmea_hdl, gps_event_handler, NULL, SCREEN_UPDATE);
}
