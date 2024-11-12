/*
 * SPDX-FileCopyrightText: 2021-2024 Espressif Systems (Shanghai) CO LTD
 *
 * SPDX-License-Identifier: CC0-1.0
 */

// This demo UI is adapted from LVGL official example: https://docs.lvgl.io/master/examples.html#loader-with-arc

#include "lvgl.h"

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



void example_lvgl_demo_ui(lv_display_t *disp)
{
    lv_obj_t *scr = lv_display_get_screen_active(disp);

    /* Alex: Try to draw a sample minimap */
    
    static lv_style_t style;
    lv_style_init(&style);
    lv_style_set_radius(&style, 50);
    lv_style_set_bg_opa(&style, LV_OPA_COVER);
    lv_style_set_bg_color(&style, lv_color_hex(0x0000FF));
    // lv_style_set_shadow_color(&style, lv_color_black());
    // lv_style_set_shadow_width(&style, 5);
    // lv_style_set_shadow_offset_x(&style, 20);
    // lv_style_set_shadow_offset_y(&style, 20);
    // lv_style_set_shadow_opa(&style, LV_OPA_50);

    lv_obj_t* me = lv_obj_create(lv_screen_active());
    lv_obj_align(me, LV_ALIGN_CENTER, -20, 0);

    lv_obj_add_style(me, &style, 0);
    lv_obj_set_size(me, 50, 50);
    lv_obj_center(me);

    lv_obj_t* me_label = lv_label_create(me);

    lv_label_set_text(me_label, "me");

    lv_obj_t* friend = lv_obj_create(lv_screen_active());
    lv_obj_t* friend_label = lv_label_create(friend);

    lv_obj_add_style(friend, &style, 0);
    lv_obj_set_size(friend, 50, 50);


    lv_obj_align(friend, LV_ALIGN_RIGHT_MID, -20, 0);
    lv_label_set_text(friend_label, "fren");
    /* end of my shenanigans */


    // btn = lv_button_create(scr);
    // lv_obj_t * lbl = lv_label_create(btn);
    // lv_label_set_text_static(lbl, LV_SYMBOL_REFRESH" ROTATE");
    // lv_obj_align(btn, LV_ALIGN_BOTTOM_LEFT, 30, -30);
    // /*Button event*/
    // lv_obj_add_event_cb(btn, btn_cb, LV_EVENT_CLICKED, disp);


    // /*Create an Arc*/
    // lv_obj_t * arc = lv_arc_create(scr);
    // lv_arc_set_rotation(arc, 270);
    // lv_arc_set_bg_angles(arc, 0, 360);
    // // lv_obj_remove_style(arc, NULL, LV_PART_KNOB);   /*Be sure the knob is not displayed*/
    // lv_obj_remove_flag(arc, LV_OBJ_FLAG_CLICKABLE);  /*To not allow adjusting by click*/
    // lv_obj_center(arc);

    // lv_anim_t a;
    // lv_anim_init(&a);
    // lv_anim_set_var(&a, arc);
    // lv_anim_set_exec_cb(&a, set_angle);
    // lv_anim_set_duration(&a, 1000);
    // lv_anim_set_repeat_count(&a, LV_ANIM_REPEAT_INFINITE);    /*Just for the demo*/
    // lv_anim_set_repeat_delay(&a, 500);
    // lv_anim_set_values(&a, 0, 100);
    // lv_anim_start(&a);
}
