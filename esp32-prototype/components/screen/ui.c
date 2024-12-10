#include "core/lv_obj_style.h"
#include "display/lv_display.h"
#include "lvgl.h"
#include "esp_log.h"
#include <math.h>
#include <stdio.h>
#include <stdlib.h>

// GPS additions
// TODO: Move to ui.h
#include "nmea_parser.h"
#include "screen.h"


/* Defines */

#define CAMPANILE_LONGITUDE  -125.25778  // campanile longitude (in degrees)
#define CAMPANILE_LATITUDE   37.87194    // campanile latitude (in degrees)
#define SATHERGATE_LONGITUDE -122.25947  // sather gate longitude (in degrees)
#define SATHERGATE_LATITUDE  37.8702180  // sather gate latitude (in degrees)
#define EARTH_RADIUS_M       6371000.0   // earth's radius in meters
#define SCREEN_RADIUS        100         // 100

/* NOTE: y values are flipped on the screen. */

/* GPS Data from right outside cory doors. */
#define CORY_DOORS_LONGITUDE -122.25826 
#define CORY_DOORS_LATITUDE    37.87535
#define CORY_DOORS_ALTITUDE   150.10001

/* TODO: Find a way to make this cleaner */

typedef struct pos {
    float lat;
    float lon;
} pos_t;

float deg_to_rad(float deg) 
{
    return deg * (M_PI / 180.0f);
}

float heading_to_rad(float heading_angle) 
{
    // This function takes the heading angle and converts it to a regular angle in radiants
    // Heading angle is confusing because 
    // (a) it is in degrees,
    // (b) it increases clockwise, and
    // (c) 0 is at the top


    float angle_deg = -heading_angle;

    float angle_rad = deg_to_rad(angle_deg);

    return angle_rad;
}


/* Converts x_ofts, y_ofs to polar then add the heading angle. Then convert back to rectangular. */
void adjust_offsets(float* x_ofs, float* y_ofs, float heading_angle)
{
    float r = sqrt((*x_ofs) * (*x_ofs) + (*y_ofs) * (*y_ofs));
    float angle1 = atan2(*y_ofs, *x_ofs);
    float angle2 = heading_to_rad(heading_angle);
    float theta = angle1 + angle2;

    *x_ofs = r * cos(theta);
    *y_ofs = r * sin(theta);
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

void draw_campanile(gps_t* global_gps, imu_data_t* global_imu)
{
    lv_obj_clean(lv_screen_active());
    lv_obj_set_style_bg_color(lv_screen_active(), lv_color_hex(0x020C0E), LV_PART_MAIN);

    /* Campanile is a target to plot, we are CORY_DOORS. */
    // Offsets of Campanile
    float x_ofs = CAMPANILE_LONGITUDE - CORY_DOORS_LONGITUDE;
    float y_ofs = CAMPANILE_LATITUDE - CORY_DOORS_LATITUDE;
    
    // Scale
    // Longitude ranges from -180 to 180
    // Latitude ranges from -90 to 90
    // hence why we scale them by different values
    float adjust_x = 100;
    float adjust_y = 100;
    x_ofs = x_ofs * adjust_x;
    y_ofs = y_ofs * adjust_y;

    x_ofs = -50.0;
    y_ofs = -50.0;
    ESP_LOGI("[DRAW_CAMPANILE]", "raw x_ofs: %0.5f\trawy_ofs: %0.5f", x_ofs, y_ofs);
    adjust_offsets(&x_ofs, &y_ofs, global_imu->heading);    
   ESP_LOGI("[DRAW_CAMPANILE]", "x_ofs: %0.3f\ty_ofs: %0.3f", x_ofs, y_ofs);
    add_bubble(x_ofs, y_ofs, "campanile");
    add_bubble(0, 0, "you");
}

void draw_sather(gps_t* global_gps, imu_data_t* global_imu) 
{
    lv_obj_clean(lv_screen_active());
    lv_obj_set_style_bg_color(lv_screen_active(), lv_color_hex(0x020C0E), LV_PART_MAIN);

    // offsets of sather gate
    float x_ofs;
    float y_ofs;

    add_bubble(0, 0, "you");

    adjust_offsets(&x_ofs, &y_ofs, global_imu->heading);    


    add_bubble(x_ofs, y_ofs, "sather");

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

void varun_ui(lv_display_t *disp)
{
    lv_obj_t *scr = lv_display_get_screen_active(disp);
    pos_t curr_pos = {0, 0};
    pos_t pos_1 = {rand() % 100, rand() % 100};
    pos_t * other_pos[2] = {&pos_1};
    display_screen(&curr_pos, other_pos, 1, 0);
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
    // draw_heading(global_imu);
    // gps_debug(global_gps);
    draw_campanile(global_gps, global_imu);
}

