/*
 * SPDX-FileCopyrightText: 2021-2024 Espressif Systems (Shanghai) CO LTD
 *
 * SPDX-License-Identifier: CC0-1.0
 */

// This demo UI is adapted from LVGL official example: https://docs.lvgl.io/master/examples.html#loader-with-arc

#include "core/lv_obj_style.h"
#include "display/lv_display.h"
#include "lvgl.h"
#include "esp_log.h"
#include <math.h>
#include <stdio.h>
#include <stdlib.h>
#include <nmea_parser.h>


/* Defines */

#define CAMPANILE_LONGITUDE  125.25778
#define CAMPANILE_LATITUDE   37.87194
#define EARTH_RADIUS_M       6371000.0
#define SCREEN_RADIUS        100

/* TODO: Find a way to make this cleaner */

typedef struct {
    float heading;
    float roll;
    float pitch;
} imu_data_t;

static imu_data_t global_imu;

static lv_display_rotation_t rotation = LV_DISP_ROTATION_270;

typedef struct pos {
    float lat;
    float lon;
} pos_t;

float deg_to_rad(float deg) {
    return deg * (M_PI / 180.0f);
}

void calculate_distance(float lat1, float lon1, float lat2, float lon2, float* x, float* y) {

    float lat1_rad = deg_to_rad(lat1);
    float lon1_rad = deg_to_rad(lon1);
    float lat2_rad = deg_to_rad(lat2);
    float lon2_rad = deg_to_rad(lon2);

    float dlat = lat2_rad - lat1_rad;
    float dlon = lon2_rad - lon1_rad;

    // Haversine formula for distance
    float a = sinf(dlat/2) * sinf(dlat/2) +
               cosf(lat1_rad) * cosf(lat2_rad) *
               sinf(dlon/2) * sinf(dlon/2);
    float central_angle = 2 * atan2(sqrt(a), sqrt(1-a));

    float total_distance = EARTH_RADIUS_M * central_angle;

    // Calculate x and y components
    // x is East-West (longitude difference)
    // y is North-South (latitude difference)
    *x = EARTH_RADIUS_M * (lon2_rad - lon1_rad) * 
                  cos((lat1_rad + lat2_rad) / 2);
    *y = EARTH_RADIUS_M * (lat2_rad - lat1_rad);

    return;
}

void add_bubble(float x_ofs, float y_ofs, char * label) {
    
    static lv_style_t style_bubble;
    lv_style_init(&style_bubble);
    lv_style_set_radius(&style_bubble, 20);
    lv_style_set_size(&style_bubble, 20, 20);
    lv_style_set_bg_opa(&style_bubble, LV_OPA_COVER);
    lv_style_set_bg_color(&style_bubble, lv_color_hex(0x23CEF1));

    lv_obj_t* person1_bubble = lv_obj_create(lv_screen_active());
    lv_obj_align(person1_bubble, LV_ALIGN_CENTER, x_ofs, y_ofs);
    lv_obj_add_style(person1_bubble, &style_bubble, 0);
    lv_obj_set_style_radius(person1_bubble, LV_RADIUS_CIRCLE, 0);
    lv_obj_set_size(person1_bubble, 20, 20);

    lv_obj_t* person1_label = lv_label_create(lv_screen_active());
    lv_label_set_text(person1_label, label);
    lv_obj_set_size(person1_label, 60, 30);
    lv_obj_set_style_text_color(person1_label, lv_color_hex(0xffffff), LV_PART_MAIN);
    lv_obj_align_to(person1_label, person1_bubble, LV_ALIGN_OUT_BOTTOM_MID, 0, 0);

}

float l2_dist(pos_t * pos) {
    return sqrt((pos->lat * pos->lat) + (pos->lon * pos->lon));
}

void display_screen(pos_t * curr_pos, pos_t * other_pos[], int num_other, float offset_angle) {
    lv_obj_clean(lv_screen_active());
    lv_obj_set_style_bg_color(lv_screen_active(), lv_color_hex(0x020C0E), LV_PART_MAIN);
    
    float max_dist_abs = 0;
    for (int i = 0; i < num_other; i++) {
        other_pos[i]->lat -= curr_pos->lat;
        other_pos[i]->lon -= curr_pos->lon;
        if (l2_dist(other_pos[i]) > max_dist_abs) {
            max_dist_abs = l2_dist(other_pos[i]);
        }
    }
    add_bubble(0, 0, "you");
    float screen_scale = SCREEN_RADIUS / max_dist_abs;
    for (int i = 0; i < num_other; i++) {
        float x_ofs = other_pos[i]->lat * screen_scale ;
        float y_ofs = -1 * other_pos[i]->lon * screen_scale;  
        char label[16];
        sprintf(label, "Fren %d", i + 1);
        ESP_LOGI("[ALEX DEMO]", "X_ofs: %f, y_ofs %f", x_ofs, y_ofs);
        add_bubble(x_ofs, y_ofs, label);
        lv_delay_ms(80);
    }

}

void display_gps_text(gps_t* global_gps) 
{
    
    char label[32];

    sprintf(label, "lat = %.03f°N", global_gps->latitude);

    add_bubble(0, -2, label);
}

void display_imu_text(imu_data_t* global_imu) 
{
    lv_obj_clean(lv_screen_active());
    lv_obj_set_style_bg_color(lv_screen_active(), lv_color_hex(0x020C0E), LV_PART_MAIN);
    
    char label[32];

    sprintf(label, "heading = %.03f°", global_imu->heading);

    add_bubble(0, 2, label);
}

void draw_campanile(gps_t* global_gps)
{
    float x_distance;
    float y_distance;

    calculate_distance(global_gps->latitude, global_gps->longitude, CAMPANILE_LATITUDE, CAMPANILE_LONGITUDE, &x_distance, &y_distance);

    // TODO: Scale distance to the size of the screen
    float x_ofs = 0;
    float y_ofs = 0;

}

void draw_heading(imu_data_t* global_imu)
{
    float r = SCREEN_RADIUS / 2;
    float theta = deg_to_rad(global_imu->heading);
    float x = r * cosf(theta);
    float y = r * sinf(theta);

    add_bubble(x, y, "N");
    ESP_LOGI("[DEBUG]", "%0.5f\t%0.5f", x, y);
    add_bubble(0, 0, "you");

}

/* Update information for the screen and display it. Gets called every 2ms. */
void update_screen(lv_display_t *disp, gps_t* global_gps, imu_data_t* global_imu)
{
    /* Reset the screen */
    lv_obj_clean(lv_screen_active());
    lv_obj_set_style_bg_color(lv_screen_active(), lv_color_hex(0x020C0E), LV_PART_MAIN);
    
    /* Write GPS data */
    // display_gps_text(global_gps);

    /* Write IMU data */
    // display_imu_text(global_imu);
    
    /* Get LoRa data */
    // TODO


    /* Draw */
    draw_heading(global_imu);
    // draw_campanile(global_gps);
    
}