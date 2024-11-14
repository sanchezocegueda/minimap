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
    return M_PI/4;
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

    static lv_style_t style_person1;
    lv_style_init(&style_person1);
    lv_style_set_radius(&style_person1, 20);
    lv_style_set_size(&style_person1, 20, 20);
    // lv_style_set_transform_rotation();
    lv_style_set_bg_opa(&style_person1, LV_OPA_COVER);
    lv_style_set_bg_color(&style_person1, lv_color_hex(0x21C143));

    static lv_style_t style_person2;
    lv_style_init(&style_person2);
    lv_style_set_radius(&style_person2, 20);
    lv_style_set_size(&style_person2, 20, 20);
    // lv_style_set_transform_rotation();
    lv_style_set_bg_opa(&style_person2, LV_OPA_COVER);
    lv_style_set_bg_color(&style_person2, lv_color_hex(0xEA3535));

    static lv_style_t style_person3;
    lv_style_init(&style_person3);
    lv_style_set_radius(&style_person3, 20);
    lv_style_set_size(&style_person3, 20, 20);
    // lv_style_set_transform_rotation();
    lv_style_set_bg_opa(&style_person3, LV_OPA_COVER);
    lv_style_set_bg_color(&style_person3, lv_color_hex(0xDAEA1D));

    static lv_style_t style_name;


    // Me

    // lv_obj_t* me_container = lv_obj_create(lv_screen_active());


    lv_obj_t* me_bubble = lv_obj_create(lv_screen_active());
    lv_obj_center(me_bubble);
    // int screen_radius = 100;// lv_display_get_horizontal_resolution((lv_display_t*) lv_screen_active());
    // float north_angle = get_angle_from_imu();
    // int32_t north_x = screen_radius * cos(north_angle);
    // int32_t north_y = -(screen_radius * sin(north_angle));
    // lv_obj_align(me_bubble, LV_ALIGN_CENTER, north_x, north_y);
    lv_obj_add_style(me_bubble, &style_bubble, 0);
    lv_obj_set_style_radius(me_bubble, LV_RADIUS_CIRCLE, 0);
    lv_obj_set_size(me_bubble, 20, 20);

    lv_obj_t* me_label = lv_label_create(lv_screen_active());
    lv_label_set_text(me_label, "You");
    lv_obj_set_size(me_label, 30, 30);
    lv_obj_set_style_text_color(me_label, lv_color_hex(0xffffff), LV_PART_MAIN);
    lv_obj_align_to(me_label, me_bubble, LV_ALIGN_OUT_BOTTOM_MID, 0, 0);

    // lv_obj_center(me_bubble);
    // int screen_radius = 100;// lv_display_get_horizontal_resolution((lv_display_t*) lv_screen_active());
    int screen_radius = 70;// lv_display_get_horizontal_resolution((lv_display_t*) lv_screen_active());
    
    /* Person 1 */

    // float north_angle = get_angle_from_imu();
    lv_obj_t* person1_bubble = lv_obj_create(lv_screen_active());
    float person1_angle = M_PI;
    int32_t person1_x = screen_radius * cos(person1_angle);
    int32_t person1_y = -(screen_radius * sin(person1_angle));
    lv_obj_align(person1_bubble, LV_ALIGN_CENTER, person1_x, person1_y);
    lv_obj_add_style(person1_bubble, &style_person1, 0);
    lv_obj_set_style_radius(person1_bubble, LV_RADIUS_CIRCLE, 0);
    lv_obj_set_size(person1_bubble, 20, 20);
    // lv_style_set_bg_color(person1_bubble, lv_color_hex(0x21C143));

    lv_obj_t* person1_label = lv_label_create(lv_screen_active());
    lv_label_set_text(person1_label, "Person 1");
    lv_obj_set_size(person1_label, 60, 30);
    lv_obj_set_style_text_color(person1_label, lv_color_hex(0xffffff), LV_PART_MAIN);
    lv_obj_align_to(person1_label, person1_bubble, LV_ALIGN_OUT_BOTTOM_MID, 0, 0);

    // lv_obj_set_style_opa(me_container, 0, 0);

    /* Person 2 */

    lv_obj_t* person2_bubble = lv_obj_create(lv_screen_active());
    float person2_angle = M_PI_4;
    int32_t person2_x = screen_radius * cos(person2_angle);
    int32_t person2_y = -(screen_radius * sin(person2_angle));
    lv_obj_align(person2_bubble, LV_ALIGN_CENTER, person2_x, person2_y);
    lv_obj_add_style(person2_bubble, &style_person2, 0);
    lv_obj_set_style_radius(person2_bubble, LV_RADIUS_CIRCLE, 0);
    lv_obj_set_size(person2_bubble, 20, 20);
    // lv_style_set_bg_color(person2_bubble, lv_color_hex(0xEA3535));

    lv_obj_t* person2_label = lv_label_create(lv_screen_active());
    // lv_obj_set_style_text_font(person2_label, &lv_font_montserrat_1, 0);
    lv_label_set_text(person2_label, "Person 2");
    lv_obj_set_size(person2_label, 70, 30);
    lv_obj_set_style_text_color(person2_label, lv_color_hex(0xffffff), LV_PART_MAIN);
    lv_obj_align_to(person2_label, person2_bubble, LV_ALIGN_OUT_BOTTOM_MID, 0, 0);

    /* Person 3 */


    lv_obj_t* person3_bubble = lv_obj_create(lv_screen_active());
    float person3_angle = 5 * M_PI_4;
    int32_t person3_x = screen_radius * cos(person3_angle);
    int32_t person3_y = -(screen_radius * sin(person3_angle));
    lv_obj_align(person3_bubble, LV_ALIGN_CENTER, person3_x, person3_y);
    lv_obj_add_style(person3_bubble, &style_person3, 0);
    lv_obj_set_style_radius(person3_bubble, LV_RADIUS_CIRCLE, 0);
    lv_obj_set_size(person3_bubble, 20, 20);
    // lv_style_set_bg_color(person3_bubble, lv_color_hex(0xDAEA1D));

    lv_obj_t* person3_label = lv_label_create(lv_screen_active());
    // lv_obj_set_style_text_font(person2_label, &lv_font_montserrat_1, 0);
    lv_label_set_text(person3_label, "Person 3");
    lv_obj_set_size(person3_label, 70, 30);
    lv_obj_set_style_text_color(person3_label, lv_color_hex(0xffffff), LV_PART_MAIN);
    lv_obj_align_to(person3_label, person3_bubble, LV_ALIGN_OUT_BOTTOM_MID, 0, 0);


    // lv_obj_align(me_label, LV_ALIGN_CENTER, -20, 0);


    // Add north indicator
    // Assume you already have an angle from the IMU



    // lv_obj_align(?, LV_ALIGN_CENTER, 20, 30);



    
}
