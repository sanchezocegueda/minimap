/*
 * SPDX-FileCopyrightText: 2021-2024 Espressif Systems (Shanghai) CO LTD
 *
 * SPDX-License-Identifier: CC0-1.0
 */

// This demo UI is adapted from LVGL official example: https://docs.lvgl.io/master/examples.html#loader-with-arc

#include "lvgl.h"
#include <math.h>

static lv_obj_t * btn;
static lv_display_rotation_t rotation = LV_DISP_ROTATION_0;

static void btn_cb(lv_event_t * e)
{
    lv_display_t *disp = lv_event_get_user_data(e);
    rotation++;
    if (rotation > LV_DISP_ROTATION_270) {
        rotation = LV_DISP_ROTATION_0;
    }
    lv_disp_set_rotation(disp, rotation);
}
static void set_angle(void * obj, int32_t v)
{
    lv_arc_set_value(obj, v);
}


float get_angle_from_imu() {
    return -M_PI;
}


void example_lvgl_demo_ui(lv_display_t *disp)
{
    lv_obj_t *scr = lv_display_get_screen_active(disp);

    lv_obj_set_style_bg_color(lv_screen_active(), lv_color_hex(0x020C0E), LV_PART_MAIN);

    /* Alex: Try to draw a sample minimap */


    // Styles
    static lv_style_t style_bubble;
    lv_style_init(&style_bubble);
    lv_style_set_radius(&style_bubble, 20);
    lv_style_set_size(&style_bubble, 20, 20);
    // lv_style_set_transform_rotation();
    lv_style_set_bg_opa(&style_bubble, LV_OPA_COVER);
    lv_style_set_bg_color(&style_bubble, lv_color_hex(0x23CEF1));

    static lv_style_t style_name;


    // Me

    // lv_obj_t* me_container = lv_obj_create(lv_screen_active());


    lv_obj_t* me_bubble = lv_obj_create(lv_screen_active());
    // lv_obj_center(me_bubble);
    int screen_radius = 100;// lv_display_get_horizontal_resolution((lv_display_t*) lv_screen_active());
    float north_angle = get_angle_from_imu();
    int32_t north_x = screen_radius * cos(north_angle);
    int32_t north_y = -(screen_radius * sin(north_angle));
    lv_obj_align(me_bubble, LV_ALIGN_CENTER, north_x, north_y);
    lv_obj_add_style(me_bubble, &style_bubble, 0);
    lv_obj_set_style_radius(me_bubble, LV_RADIUS_CIRCLE, 0);
    lv_obj_set_size(me_bubble, 20, 20);

    lv_obj_t* me_label = lv_label_create(lv_screen_active());
    lv_label_set_text(me_label, "me");
    lv_obj_set_size(me_label, 30, 30);
    lv_obj_set_style_text_color(me_label, lv_color_hex(0xffffff), LV_PART_MAIN);
    lv_obj_align_to(me_label, me_bubble, LV_ALIGN_OUT_TOP_LEFT, 20, 20);

    // lv_obj_set_style_opa(me_container, 0, 0);


    // Friend

    static lv_style_t style_friend;
    lv_style_init(&style_friend);
    lv_style_set_radius(&style_friend, 25);
    lv_style_set_bg_opa(&style_friend, LV_OPA_COVER);
    lv_style_set_bg_color(&style_friend, lv_color_hex(0xE92626));


    // lv_obj_align(me_label, LV_ALIGN_CENTER, -20, 0);


    // Add north indicator
    // Assume you already have an angle from the IMU



    // lv_obj_align(?, LV_ALIGN_CENTER, 20, 30);



    
}
