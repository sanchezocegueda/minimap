#ifndef __SCREEN_H__
#define __SCREEN_H__

#include <stdio.h>
#include <unistd.h>
#include <sys/lock.h>
#include <sys/param.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_timer.h"
#include "esp_lcd_panel_io.h"
#include "esp_lcd_panel_vendor.h"
#include "esp_lcd_panel_ops.h"
#include "driver/gpio.h"
#include "driver/spi_master.h"
#include "esp_err.h"
#include "esp_log.h"
#include "lvgl.h"
#include "misc/lv_types.h"

#include "esp_lcd_gc9a01.h"

// Using SPI2 in the example, Luca: this is the software SPI interface
#define LCD_HOST  SPI2_HOST

#define LCD_PIXEL_CLOCK_HZ     (20 * 1000 * 1000)
#define LCD_BK_LIGHT_ON_LEVEL  1
#define LCD_BK_LIGHT_OFF_LEVEL !LCD_BK_LIGHT_ON_LEVEL

// Pins for our screen
#define PIN_NUM_SCLK        (16)
#define PIN_NUM_MOSI        (17)
#define PIN_NUM_LCD_DC      (4)
#define PIN_NUM_LCD_RST     (15)
#define PIN_NUM_LCD_CS      (0)

// The pixel number in horizontal and vertical
#define LCD_H_RES           (240)
#define LCD_V_RES           (240)

// Bit number used to represent command and parameter
#define LCD_CMD_BITS           8
#define LCD_PARAM_BITS         8

// TODO: Parameters we can tweak
#define LVGL_DRAW_BUF_LINES    20 // number of display lines in each draw buffer
#define LVGL_TICK_PERIOD_MS    2
#define LVGL_TASK_MAX_DELAY_MS 500
#define LVGL_TASK_MIN_DELAY_MS 1
#define LVGL_TASK_STACK_SIZE   (4 * 1024)
#define LVGL_TASK_PRIORITY     2


void start_screen(void);

void increase_lvgl_tick(void *arg);

void lvgl_flush_cb(lv_display_t *disp, const lv_area_t *area, uint8_t *px_map);

void rotate_esp_lcd(lv_display_t *disp);

bool notify_lvgl_flush_ready(esp_lcd_panel_io_handle_t panel_io, esp_lcd_panel_io_event_data_t *edata, void *user_ctx);

void lvgl_timer_task(void* pvParam);

void clear_screen(void);

#endif
