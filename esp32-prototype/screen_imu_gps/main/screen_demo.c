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

/* TODO: Find a way to make this cleaner */

typedef struct {
    float heading;
    float roll;
    float pitch;
} imu_data_t;

static imu_data_t global_imu;

static const char* DEMO_TAG = "[DEMO]";

static lv_obj_t * btn;
static lv_display_rotation_t rotation = LV_DISP_ROTATION_270;
static int screen_radius = 100;
typedef struct pos {
    float lat;
    float lon;
} pos_t;

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
    float screen_scale = screen_radius / max_dist_abs;
    for (int i = 0; i < num_other; i++) {
        float x_ofs = other_pos[i]->lat * screen_scale ;
        float y_ofs = -1 * other_pos[i]->lon * screen_scale;  
        char label[16];
        sprintf(label, "Fren %d", i + 1);
        ESP_LOGI(DEMO_TAG, "X_ofs: %f, y_ofs %f", x_ofs, y_ofs);
        add_bubble(x_ofs, y_ofs, label);
        lv_delay_ms(80);
    }

}

void display_gps_text(gps_t* global_gps) 
{
    lv_obj_clean(lv_screen_active());
    lv_obj_set_style_bg_color(lv_screen_active(), lv_color_hex(0x020C0E), LV_PART_MAIN);
    
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


static void btn_cb(lv_event_t * e)
{
    lv_display_t *disp = lv_event_get_user_data(e);
    rotation++;
    if (rotation > LV_DISP_ROTATION_270) {
        rotation = LV_DISP_ROTATION_0;
    }
    lv_disp_set_rotation(disp, rotation);
}

void update_screen(lv_display_t* disp, gps_t* global_gps)
{
    // TODO
}

void screen_demo_ui(lv_display_t *disp, gps_t* global_gps, imu_data_t* global_imu)
{
    lv_disp_set_rotation(disp, rotation);
    lv_obj_t *scr = lv_display_get_screen_active(disp);
    pos_t curr_pos = {0, 0};
    pos_t pos_1 = {rand() % 100, rand() % 100};
    pos_t * other_pos[2] = {&pos_1};
    // display_screen(&curr_pos, other_pos, 1, 0);
    ESP_LOGI("DEBUG", " => \r\n"
                 "\t\t\t\t\t\tlatitude   = %.05f°N\r\n"
                 "\t\t\t\t\t\tlongitude = %.05f°E\r\n"
                 "\t\t\t\t\t\taltitude   = %.02fm\r\n"
                 "\t\t\t\t\t\tspeed      = %fm/s",
                //  gps->date.year + YEAR_BASE, gps->date.month, gps->date.day,
                //  gps->tim.hour + TIME_ZONE, gps->tim.minute, gps->tim.second,
                 global_gps->latitude, global_gps->longitude, global_gps->altitude, global_gps->speed);
    // display_gps_text(global_gps);
    display_imu_text(global_imu);
    
}