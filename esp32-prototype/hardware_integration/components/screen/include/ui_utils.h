#ifndef __UI_UTILS_H__
#define __UI_UTILS_H__

#include <stdint.h>
#include <stdio.h>
#include <stdlib.h>
#include <math.h>

#include "core/lv_obj_style.h"
#include "display/lv_display.h"
#include "esp_log.h"
#include "lvgl.h"

#include "freertos/FreeRTOS.h"
#include "freertos/queue.h"
#include "freertos/task.h"

// GPS additions
#include "nmea_parser.h"
#include "screen.h"

#include "mpu9250.h"

/* Defines */
#define MAX_DISPLAY_RADIUS 100
#define MIN_DISPLAY_RADIUS 20
#define SCREEN_SCALE 2

/* Global variables */
extern _lock_t lvgl_api_lock;
extern lv_disp_t* display;

extern QueueHandle_t screen_lora_event_queue;
extern QueueHandle_t button_event_queue;

extern coordinates_t other_pos;
extern coordinates_t curr_pos;

/* Euclidean Coordinate */
typedef struct pos {
  float x;
  float y;
} pos_t;

typedef struct imu_data {
    float heading;
    float roll;
    float pitch;
} imu_data_t;

typedef struct lora_packet {
  bool valid;      // 0 for tx, 1 for rx
  uint32_t counter_val; // gps position
} lora_packet_t;

typedef struct lora_gps_packet {
  bool valid;      // 0 for tx, 1 for rx
  coordinates_t curr_gps_pos; // gps position
} lora_gps_packet_t;

/* pvParameter for app_main to pass to xCreateTask for the screen */
typedef struct screen_task_params {
    imu_data_t* global_imu;
    nmea_parser_handle_t nmea_hndl;
    QueueHandle_t* screen_lora_event_queue;
} screen_task_params_t;

/* pvParameter for app_main to pass to xCreateTask for the calibration ui */
typedef struct calibrate_screen_params {
  calibration_t* cal_x; /* Which MPU calibration to overwrite */
  QueueHandle_t* button_event_queue; /* To block on button presses */
} calibrate_screen_params_t;




void draw_bubble(pos_t *position, char *label);

void screen_main_task(void* arg);

/* Function to render task_both sending a counter and receiving a counter. */
void render_counter();

// Display text on screen for s seconds
void display_text(char* text);
void display_line_0(char* text);
void display_line_1(char* text);
void display_line_2(char* text);
void display_line_3(char* text);

void calibrate_task(void *pvParam);

#endif