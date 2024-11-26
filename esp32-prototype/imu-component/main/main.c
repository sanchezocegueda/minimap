#include <stdio.h>
#include <stdbool.h>
#include <string.h>

#include "esp_timer.h"
#include "esp_log.h"
#include "esp_err.h"

#include "Fusion.h"
#include "mpu9250.h"



#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/queue.h"

#include "driver/i2c.h"

#include "calibrate.h"
#include "common.h"


#define I2C_MASTER_NUM I2C_NUM_0 /*!< I2C port number for master dev */

calibration_t cal = {
    .mag_offset = {.x = -5.449219, .y = -4.875000, .z = 13.476562},
    .mag_scale = {.x = 1.175105, .y = 0.930902, .z = 0.930418},
    .gyro_bias_offset = {.x = -0.940880, .y = -0.704113, .z = -1.574584},
    .accel_offset = {.x = 0.033275, .y = 0.086760, .z = 0.103982},
    .accel_scale_lo = {.x = 1.013889, .y = 1.026322, .z = 1.050056},
    .accel_scale_hi = {.x = -0.982644, .y = -0.976446, .z = -0.974061}};


void run_imu()
{
    i2c_mpu9250_init(&cal);

    // Define calibration (replace with actual calibration data if available)
    const FusionMatrix gyroscopeMisalignment = {.element = {1.0f, 0.0f, 0.0f, 0.0f, 1.0f, 0.0f, 0.0f, 0.0f, 1.0f}};
    const FusionVector gyroscopeSensitivity = {{1.0f, 1.0f, 1.0f}};
    const FusionVector gyroscopeOffset = {{0.0f, 0.0f, 0.0f}};
    const FusionMatrix accelerometerMisalignment = {.element = {1.0f, 0.0f, 0.0f, 0.0f, 1.0f, 0.0f, 0.0f, 0.0f, 1.0f}};
    const FusionVector accelerometerSensitivity = {{1.0f, 1.0f, 1.0f}};
    const FusionVector accelerometerOffset = {{0.0f, 0.0f, 0.0f}};
    const FusionMatrix softIronMatrix = {.element = {1.0f, 0.0f, 0.0f, 0.0f, 1.0f, 0.0f, 0.0f, 0.0f, 1.0f}};
    const FusionVector hardIronOffset = {{0.0f, 0.0f, 0.0f}};

    // Initialise algorithms
    FusionOffset offset;
    FusionAhrs ahrs;

    FusionOffsetInitialise(&offset, SAMPLE_FREQ_Hz);
    FusionAhrsInitialise(&ahrs);

    // Set AHRS algorithm settings
    const FusionAhrsSettings settings = {
            .convention = FusionConventionNwu,
            .gain = 0.5f,
            .gyroscopeRange = 2000.0f, /* replace this with actual gyroscope range in degrees/s */
            .accelerationRejection = 10.0f,
            .magneticRejection = 10.0f,
            .recoveryTriggerPeriod = 5 * SAMPLE_FREQ_Hz, /* 5 seconds */
    };
    FusionAhrsSetSettings(&ahrs, &settings);

    // This loop should repeat each time new gyroscope data is available
    while (true) {
        vector_t va, vg, vm;
        ESP_ERROR_CHECK(get_accel_gyro_mag(&va, &vg, &vm));


        // Acquire latest sensor data
        int64_t timestamp = esp_timer_get_time();  // replace this with actual gyroscope timestamp
        FusionVector gyroscope = {{vg.x, vg.y, vg.z}}; // replace this with actual gyroscope data in degrees/s
        FusionVector accelerometer = {{va.x, va.y, va.z}}; // replace this with actual accelerometer data in g
        FusionVector magnetometer = {{vm.x, vm.y, vm.z}}; // replace this with actual magnetometer data in arbitrary units

        // Apply calibration
        gyroscope = FusionCalibrationInertial(gyroscope, gyroscopeMisalignment, gyroscopeSensitivity, gyroscopeOffset);
        accelerometer = FusionCalibrationInertial(accelerometer, accelerometerMisalignment, accelerometerSensitivity, accelerometerOffset);
        magnetometer = FusionCalibrationMagnetic(magnetometer, softIronMatrix, hardIronOffset);

        // Update gyroscope offset correction algorithm
        gyroscope = FusionOffsetUpdate(&offset, gyroscope);

        // Calculate delta time (in seconds) to account for gyroscope sample clock error
        static int64_t previousTimestamp;
        const float deltaTime = (float) ((timestamp - previousTimestamp) / 10E6);
        previousTimestamp = timestamp;

        // Update gyroscope AHRS algorithm
        FusionAhrsUpdate(&ahrs, gyroscope, accelerometer, magnetometer, deltaTime);

        // Print algorithm outputs
        const FusionEuler euler = FusionQuaternionToEuler(FusionAhrsGetQuaternion(&ahrs));
        const FusionVector earth = FusionAhrsGetEarthAcceleration(&ahrs);

        printf("Roll %0.1f, Pitch %0.1f, Yaw %0.1f, X %0.1f, Y %0.1f, Z %0.1f\n",
               euler.angle.roll, euler.angle.pitch, euler.angle.yaw,
               earth.axis.x, earth.axis.y, earth.axis.z);

        vTaskDelay(0); // WDT
    }
}

void app_main(void)
{
  xTaskCreate(run_imu, "imu_task", 4096, NULL, 10, NULL);
}
