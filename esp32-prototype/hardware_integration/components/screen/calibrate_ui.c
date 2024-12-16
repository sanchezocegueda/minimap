/* Custom functions for calibrating and displaying info on the screen */
#include "ui_utils.h"
#include <math.h>

// Calibration headers
#include "esp_system.h"
#include "esp_err.h"
#include "esp_task_wdt.h"
#include "mpu9250.h"
#include "common.h"

#include "buttons.h"

const char *TAG = "calibrate";

/* Delay the task for X seconds */
void wait(int x)
{
  vTaskDelay(x * 1000 / portTICK_PERIOD_MS);
}

/* Default calibration to start from */
static calibration_t cal = {
    .mag_offset = {.x = 0.0, .y = 0.0, .z = 0.0},
    .mag_scale = {.x = 1.0, .y = 1.0, .z = 1.0},
    .accel_offset = {.x = 0.0, .y = 0.0, .z = 0.0},
    .accel_scale_lo = {.x = -1.0, .y = -1.0, .z = -1.0},
    .accel_scale_hi = {.x = 1.0, .y = 1.0, .z = 1.0},
    .gyro_bias_offset = {.x = 0.0, .y = 0.0, .z = 0.0}
};


void countdown(int x)
{
  char buf[32];
  for (int i = x; i >= 0; i -= 1)
  {
    sprintf(buf, "Starting in %d seconds", i);
    display_text(buf);
    wait(1);
  }
}


/**
 * 
 * GYROSCOPE
 * 
 * 
 * Calibrate the gyro.  The device needs to remain still during calibration.  The calibration will
 * be applied to the gyro.  This is only simple calibration for Gyro bias for when the Gyro is still.
 * More sophisticated calibration tools can be applied.
 *
 * NOTE: The Gyro must not be moved during this process.
 *
 */

// TODO: move to header
const int NUM_GYRO_READS = 5000;

void calibrate_gyro_with_output(calibration_t* cal_mpu_x)
{
  wait(1);
  display_text("--- GYRO CALIBRATION ---");
  wait(1);
  countdown(5);
  display_text("Keep the MPU very still.");
  wait(1);
  display_text("Calculating gyroscope bias...");
  wait(1);

  vector_t vg_sum;
  vg_sum.x = 0.0;
  vg_sum.y = 0.0;
  vg_sum.z = 0.0;
  for (int i = 0; i < NUM_GYRO_READS; i += 1)
  {
    vector_t vg;
    ESP_ERROR_CHECK(get_gyro(&vg));

    vg_sum.x += vg.x;
    vg_sum.y += vg.y;
    vg_sum.z += vg.z;

    // Make the WDT happy
    if (i % 100 == 0)
      vTaskDelay(0);

    imu_pause();
  }
  vg_sum.x /= -NUM_GYRO_READS;
  vg_sum.y /= -NUM_GYRO_READS;
  vg_sum.z /= -NUM_GYRO_READS;

  // Copy the values into our struct
  cal_mpu_x->gyro_bias_offset = vg_sum;
  
  ESP_LOGW("[CALIBRATION]\n", "    .gyro_bias_offset = {.x = %f, .y = %f, .z = %f}\n", vg_sum.x, vg_sum.y, vg_sum.z);

  display_text("Finished calibrating gyroscope");
  wait(1);
}

/**
 * 
 * ACCELEROMETER 
 * 
 * 
 * Calibrate the Accelerometer.  This device will need to be rotated with the X, Y and Z axes up and down.  The axis
 * you point up/down will be calibrated against gravity (so you must have it vertical).  You may want to hold it against
 * a wall or similar.  While the one axis is being calibrated against gravity, the other two axes will be perpendicular
 * to gravity, so will read near zero, this value will be used as the offset.
 *
 * The scaling is simple linear scaling, based on the common formular for a line, y = m * x + c, where y is our scaled
 * and offset result, while x is the raw value.  This formular is actually applied in the main mpu9250.js file.  But
 * this calibration process outputs those values.
 */

// TODO: move to header ======================================
#define NUM_ACCEL_READS (1000.0)

#define X_AXIS (0)
#define Y_AXIS (1)
#define Z_AXIS (2)
const char *axes[] = {"X", "Y", "Z"};

#define DIR_UP (0)
#define DIR_DOWN (1)
const char *directions[] = {
    "up",
    "down"};
// TODO: move to header ======================================

vector_t offset = {.x = 0, .y = 0, .z = 0};
vector_t scale_lo = {.x = 0, .y = 0, .z = 0};
vector_t scale_hi = {.x = 0, .y = 0, .z = 0};

/**
 * This will syncronuously read the accel data from MPU9250.  It will gather the offset and scalar values.
 */
void calibrate_accel_axis(int axis, int dir)
{
  vector_t va;

  display_text("Reading values - hold still");
  wait(1);
  for (int i = 0; i < NUM_ACCEL_READS; i++)
  {
    get_accel(&va);

    if (axis == X_AXIS)
    {
      if (dir == DIR_UP)
      {
        scale_lo.x += va.x;
      }
      else
      {
        scale_hi.x += va.x;
      }
    }
    else
    {
      offset.y += va.y;
      offset.z += va.z;
    }

    if (axis == Y_AXIS)
    {
      if (dir == DIR_UP)
      {
        scale_lo.y += va.y;
      }
      else
      {
        scale_hi.y += va.y;
      }
    }
    else
    {
      offset.x += va.x;
      offset.z += va.z;
    }

    if (axis == Z_AXIS)
    {
      if (dir == DIR_UP)
      {
        scale_lo.z += va.z;
      }
      else
      {
        scale_hi.z += va.z;
      }
    }
    else
    {
      offset.x += va.x;
      offset.y += va.y;
    }

    vTaskDelay(5 / portTICK_PERIOD_MS);
  }
}



/**
 * Set up the next capture for an axis and a direction (up / down).
 */
void run_next_capture(int axis, int dir)
{
  char buf[32];
  sprintf(buf, "Point the %s axis arrow %s.", axes[axis], directions[dir]);
  display_text(buf);
  wait(3);

  countdown(5);
  calibrate_accel_axis(axis, dir);
}


/* Modified version of calibrate_accel that stores the calibration data persistently. */
void calibrate_accel_with_output(calibration_t* cal_mpu_x)
{

  display_text("--- ACCEL CALIBRATION ---");
  wait(1);

  run_next_capture(X_AXIS, DIR_UP);
  run_next_capture(X_AXIS, DIR_DOWN);
  run_next_capture(Y_AXIS, DIR_UP);
  run_next_capture(Y_AXIS, DIR_DOWN);
  run_next_capture(Z_AXIS, DIR_UP);
  run_next_capture(Z_AXIS, DIR_DOWN);

  offset.x /= (NUM_ACCEL_READS * 4.0);
  offset.y /= (NUM_ACCEL_READS * 4.0);
  offset.z /= (NUM_ACCEL_READS * 4.0);
  scale_lo.x /= NUM_ACCEL_READS;
  scale_lo.y /= NUM_ACCEL_READS;
  scale_lo.z /= NUM_ACCEL_READS;
  scale_hi.x /= NUM_ACCEL_READS;
  scale_hi.y /= NUM_ACCEL_READS;
  scale_hi.z /= NUM_ACCEL_READS;

  cal_mpu_x->accel_offset.x = offset.x;
  cal_mpu_x->accel_offset.y = offset.y;
  cal_mpu_x->accel_offset.z = offset.z;

  cal_mpu_x->accel_scale_lo.x = scale_lo.x;
  cal_mpu_x->accel_scale_lo.y = scale_lo.y;
  cal_mpu_x->accel_scale_lo.z = scale_lo.z;
  
  cal_mpu_x->accel_scale_hi.x = scale_hi.x;
  cal_mpu_x->accel_scale_hi.y = scale_hi.y;
  cal_mpu_x->accel_scale_hi.z = scale_hi.z;

  ESP_LOGW("[CALIBRATION]\n", "    .accel_offset = {.x = %f, .y = %f, .z = %f},\n    .accel_scale_lo = {.x = %f, .y = %f, .z = %f},\n    .accel_scale_hi = {.x = %f, .y = %f, .z = %f},\n",
         offset.x, offset.y, offset.z,
         scale_lo.x, scale_lo.y, scale_lo.z,
         scale_hi.x, scale_hi.y, scale_hi.z);

}

/**
 *
 * MAGNETOMETER
 * 
 * 
 * Once the calibration is started you will want to move the sensor around all axes.  What we want is to find the
 * extremes (min/max) of the x, y, z values such that we can find the offset and scale values.
 *
 * These calibration calculations are based on this page:
 * http://www.camelsoftware.com/2016/03/13/imu-maths-calculate-orientation-pt3/
 */

// TODO: Move to header
// #define MIN(a, b) (a < b ? a : b)
// #define MAX(a, b) (a > b ? a : b)

void calibrate_mag_with_output(calibration_t* cal_mpu_x)
{

  vector_t v_min = {
      .x = 9.9e99,
      .y = 9.9e99,
      .z = 9.9e99};
  vector_t v_max = {
      .x = -9.9e99,
      .y = -9.9e99,
      .z = -9.9e99};

  // TODO: Move to header
  const int NUM_MAG_READS = 2000;

  clear_screen();

  char buf0[96];
  sprintf(buf0, "Rotate the magnometer around all 3 axes, until the min and max values don't change anymore.");
  char buf1[96];
  sprintf(buf1, "  x        y        z      min x     min y     min z     max x     max y     max z");
  display_line_0(buf0);
  display_line_1(buf1);
  wait(1);

  char buf2[96];

  for (int i = 0; i < NUM_MAG_READS; i += 1)
  {
    vector_t vm;
    get_mag(&vm);
    v_min.x = MIN(v_min.x, vm.x);
    v_min.y = MIN(v_min.y, vm.y);
    v_min.z = MIN(v_min.z, vm.z);
    v_max.x = MAX(v_max.x, vm.x);
    v_max.y = MAX(v_max.y, vm.y);
    v_max.z = MAX(v_max.z, vm.z);


    sprintf(buf2, " %0.3f    %0.3f    %0.3f    %0.3f   %0.3f   %0.3f   %0.3f   %0.3f   %0.3f", vm.x, vm.y, vm.z, v_min.x, v_min.y, v_min.z, v_max.x, v_max.y, v_max.z);
    // clear_screen();
    // display_line_0(buf0);
    // display_line_1(buf1);
    // display_line_2(buf2);
    // wait(1);

    vTaskDelay(10 / portTICK_PERIOD_MS);
  }

  vector_t v_avg = {
      .x = (v_max.x - v_min.x) / 2.0,
      .y = (v_max.y - v_min.y) / 2.0,
      .z = (v_max.z - v_min.z) / 2.0};
  float avg_radius = (v_avg.x + v_avg.y + v_avg.z) / 3.0;
  vector_t v_scale = {
      .x = avg_radius / v_avg.x,
      .y = avg_radius / v_avg.y,
      .z = avg_radius / v_avg.z};

  cal_mpu_x->mag_offset.x = (v_min.x + v_max.x) / 2;
  cal_mpu_x->mag_offset.y = (v_min.y + v_max.y) / 2;
  cal_mpu_x->mag_offset.z = (v_min.z + v_max.z) / 2;

  cal_mpu_x->mag_scale.x = v_scale.x;
  cal_mpu_x->mag_scale.y = v_scale.y;
  cal_mpu_x->mag_scale.z = v_scale.z;


  ESP_LOGW("[CALIBRATION]\n", "    .mag_offset = {.x = %f, .y = %f, .z = %f},\n", (v_min.x + v_max.x) / 2, (v_min.y + v_max.y) / 2, (v_min.z + v_max.z) / 2);
  ESP_LOGW("[CALIBRATION]\n", "    .mag_scale = {.x = %f, .y = %f, .z = %f},\n", v_scale.x, v_scale.y, v_scale.z);

  display_text("Magnetometer calibration complete");
  wait(1);
}


/* Prompts the user for calibration, and blocks until a button is pressed.
If the right button is pressed, we calibrate, render the steps for calibration, and
write the values into the provided cal pointer in pvParam. NOTE: Not actually a task.
  - Needs pointer to cal struct
  - Needs button event queue (global for now)
*/
void calibrate_task(void *pvParam){
  calibrate_screen_params_t* args = (calibrate_screen_params_t*) pvParam;
  calibration_t* imu_cal = args->cal_x;
  *imu_cal = cal;

  /* Block until a button press. */
  minimap_button_event_t press;
  ESP_LOGI("[CALIBRATION]", "Waiting for user to press a button to calibrate");
  if (xQueueReceive(button_event_queue, &press, portMAX_DELAY) == pdPASS && press == LEFT_PRESS) {
    /* On left button press, we don't calibrate */
      return;
  }
  /* Right button press (we do want to calibrate) */
  /* lvgl_timer_task is to count lvgl ticks while the calibration task runs */
  TaskHandle_t timer_handle;
  xTaskCreate(lvgl_timer_task, "lvgl timer task", 4096, NULL, 8, &timer_handle);

  /* Init IMU */
  init_imu(imu_cal);

  /* Perform screen calibration */
  calibrate_gyro_with_output(imu_cal);
  calibrate_accel_with_output(imu_cal);
  calibrate_mag_with_output(imu_cal);
  clear_screen();
  /* Delete lvgl timer task, screen_main_task will need to handle lv timers with lv_timer_handler() */
  vTaskDelete(timer_handle);
}
