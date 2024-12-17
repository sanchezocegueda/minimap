#include "ui_utils.h"
/* NOTE: y values are flipped on the screen. */

/* Defines */
#define CAMPANILE_LONGITUDE -122.25777  // campanile longitude (in degrees)
#define CAMPANILE_LATITUDE 37.87194     // campanile latitude (in degrees)
#define SATHERGATE_LONGITUDE -122.25947 // sather gate longitude (in degrees)
#define SATHERGATE_LATITUDE 37.8702180  // sather gate latitude (in degrees)
#define EARTH_RADIUS_M 6371000.0        // earth's radius in meters
#define SCREEN_SLEEP_ORIENTATION_THRESH 20 // absolute difference in degrees from level plane that triggers screen off


/* GPS Data from right outside cory doors. */
#define CORY_DOORS_LONGITUDE -122.25798
#define CORY_DOORS_LATITUDE 37.87535
#define CORY_DOORS_ALTITUDE 150.10001

const char* SCREEN_TAG = "[SCREEN]";

/* Get distance from origin */
float l2_dist(pos_t *pos) {
  return sqrt((pos->y * pos->y) + (pos->x * pos->x));
}

/* Converts angle in degrees to radians */
float deg_to_rad(float deg) { return deg * (M_PI / 180.0f); }


/* Global variables for gps coordinates */
gps_output_t curr_out;
gps_output_t other_outc;

/**
 * @brief Gets the euclidean distance between two GPS coordinate pairs
 * @param coords1 GPS coordinates
 * @param coords2 GPS coordinates
 * @return relative distance from coords1 to coords2
 */
pos_t get_relative_pos(coordinates_t *coords1, coordinates_t *coords2) {
  /* Convert coordinates from degrees to radians */
  float lat1_rad = deg_to_rad(coords1->lat);
  float lon1_rad = deg_to_rad(coords1->lon);
  float lat2_rad = deg_to_rad(coords2->lat);
  float lon2_rad = deg_to_rad(coords2->lon);

  /* Get distance between latitudinal and longitudinal angles */
  float d_lat = lat2_rad - lat1_rad;
  float d_lon = lon2_rad - lon1_rad;

  /* Haversine formula for distance */
  float a = sinf(d_lat / 2) * sinf(d_lat / 2) +
            cosf(lat1_rad) * cosf(lat2_rad) * sinf(d_lon / 2) * sinf(d_lon / 2);

  float central_angle = 2 * atan2(sqrt(a), sqrt(1 - a));

  float total_distance = EARTH_RADIUS_M * central_angle;

  /* Calculate x and y components
     x is East-West (longitude difference)
     y is North-South (latitude difference) */
  float x_dist = EARTH_RADIUS_M * (lon2_rad - lon1_rad) * cos((lat1_rad + lat2_rad) / 2);
  float y_dist = EARTH_RADIUS_M * (lat2_rad - lat1_rad);

  pos_t dist_relative = {x_dist, y_dist};

  return dist_relative;
}

/**
 * @brief Rotates pos by heading_angle and scales to screen.
 * @param pos Expects pos in meters. MUTATED
 * @param heading_angle expects angle in degrees
 *
 * Converts pos to polar then add the heading angle. Then convert back to
 * rectangular.
 */
void transform_to_screen(pos_t *pos, float heading_angle) {
  /* Calculate rotation angle relative to heading */
  float r = sqrt((pos->x) * (pos->x) + (pos->y) * (pos->y));
  float angle1 = atan2(pos->y, pos->x);
  float angle2 = deg_to_rad(
      heading_angle); // TODO: Verify we didn't need the negative @Alex
  float theta = angle1 + angle2;

  /* Scale from meters to screen coordinates */
  r *= SCREEN_SCALE;

  /* Clamp to screen edge */
  r = fmaxf(fminf(r, MAX_DISPLAY_RADIUS), MIN_DISPLAY_RADIUS);

  /* Apply rotation */
  pos->x = r * cos(theta);
  pos->y = r * sin(theta);

  /* Transform into screen coordinates */
  pos->x = pos->x;
  pos->y = -1 * pos->y;
}

/**
 * @brief draw a labeled bubble on screen.
 */
void draw_bubble(pos_t *position, char *label) {
  /* Initialize style for the bubble */
  _lock_acquire(&lvgl_api_lock);
  static lv_style_t style_bubble;
  lv_style_init(&style_bubble);
  lv_style_set_radius(&style_bubble, 20); // Set the radius for rounded corners
  lv_style_set_bg_opa(&style_bubble, LV_OPA_COVER);
  lv_style_set_bg_color(&style_bubble, lv_color_hex(0x23CEF1));

  /* Create bubble object */
  lv_obj_t *bubble_obj =
      lv_obj_create(lv_scr_act());                // Add to the active screen
  lv_obj_set_size(bubble_obj, 20, 20);            // Set size of the bubble
  lv_obj_add_style(bubble_obj, &style_bubble, 0); // Add the style to the bubble
  lv_obj_align(bubble_obj, LV_ALIGN_CENTER, position->x, position->y); // Align the bubble

  /* Create label object */
  lv_obj_t *label_obj =
      lv_label_create(lv_scr_act());   // Add label to the active screen
  lv_label_set_text(label_obj, label); // Set the label text
  lv_obj_set_style_text_color(label_obj, lv_color_hex(0xFFFFFF), LV_PART_MAIN); // Set text color
  lv_obj_align_to(label_obj, bubble_obj, LV_ALIGN_OUT_BOTTOM_MID, 0, 10); // Align the label to bubble
  _lock_release(&lvgl_api_lock);
}

/**
 * @brief draw a labeled bubble on screen.
 */
void draw_bubble(pos_t *position, char *label) {
  /* Initialize style for the bubble */
  _lock_acquire(&lvgl_api_lock);
  static lv_style_t style_other;
  lv_style_init(&style_other);
  lv_style_set_radius(&style_other, 20); // Set the radius for rounded corners
  lv_style_set_bg_opa(&style_other, LV_OPA_COVER);
  lv_style_set_bg_color(&style_other, lv_color_hex(0x03AC13));

  /* Create bubble object */
  lv_obj_t *bubble_obj =
      lv_obj_create(lv_scr_act());                // Add to the active screen
  lv_obj_set_size(bubble_obj, 20, 20);            // Set size of the bubble
  lv_obj_add_style(bubble_obj, &style_other, 0); // Add the style to the bubble
  lv_obj_align(bubble_obj, LV_ALIGN_CENTER, position->x, position->y); // Align the bubble

  /* Create label object */
  lv_obj_t *label_obj =
      lv_label_create(lv_scr_act());   // Add label to the active screen
  lv_label_set_text(label_obj, label); // Set the label text
  lv_obj_set_style_text_color(label_obj, lv_color_hex(0xFFFFFF), LV_PART_MAIN); // Set text color
  lv_obj_align_to(label_obj, bubble_obj, LV_ALIGN_OUT_BOTTOM_MID, 0, 10); // Align the label to bubble
  _lock_release(&lvgl_api_lock);
}

/**
 * @brief draw a labeled bubble on screen.
 */
void draw_north(pos_t *position, char *label) {
  /* Initialize style for the bubble */
  _lock_acquire(&lvgl_api_lock);
  static lv_style_t style_north;
  lv_style_init(&style_north);
  lv_style_set_radius(&style_north, 10); // Set the radius for rounded corners
  lv_style_set_bg_opa(&style_north, LV_OPA_COVER);
  lv_style_set_bg_color(&style_north, lv_color_hex(0xB90E0A));

  /* Create bubble object */
  lv_obj_t *bubble_obj =
      lv_obj_create(lv_scr_act());                // Add to the active screen
  lv_obj_set_size(bubble_obj, 10, 10);            // Set size of the bubble
  lv_obj_add_style(bubble_obj, &style_north, 0); // Add the style to the bubble
  lv_obj_align(bubble_obj, LV_ALIGN_CENTER, position->x, position->y); // Align the bubble

  /* Create label object */
  lv_obj_t *label_obj =
      lv_label_create(lv_scr_act());   // Add label to the active screen
  lv_label_set_text(label_obj, label); // Set the label text
  lv_obj_set_style_text_color(label_obj, lv_color_hex(0xFFFFFF), LV_PART_MAIN); // Set text color
  lv_obj_align_to(label_obj, bubble_obj, LV_ALIGN_OUT_BOTTOM_MID, 0, 10); // Align the label to bubble
  _lock_release(&lvgl_api_lock);
}

/**
 * @brief Draw indicator pointing toward north
 * @param heading_angle expects the heading in degrees
 */
void draw_north_indicator(float heading_angle) {

  float angle = deg_to_rad(-90 - heading_angle);
  float x = cosf(angle) * MAX_DISPLAY_RADIUS;
  float y = sinf(angle) * MAX_DISPLAY_RADIUS;

  pos_t north_pos;
  north_pos.x = x;
  north_pos.y = y;

  char buf[16];
  sprintf(buf, "%d°", (int) heading_angle);

  draw_north(&north_pos, buf);
}

/**
 * @brief Displays every GPS point-of-interest on the screen.
 * @param  f Current user GPS position
 * @param other_pos Array of other points of interest
 * @param num_other Length of other_pos array
 * @param offset_angle Current heading relative to north
 */
void display_screen(coordinates_t *curr_pos, coordinates_t *other_pos,
                    int num_other, float offset_angle) {
  pos_t origin = {0, 0};
  char our_label[32];
  sprintf(our_label, "%.5f °N\n%.5f °E", curr_pos->lat, curr_pos->lon);
  draw_bubble(&origin, our_label);

  // TODO: Appears to be 90 degrees off for some reason? Needs debugging
  // TODO: Make this thread-safe, no lvgl function can be called without a lock
  draw_north_indicator(offset_angle);
  // ESP_LOGI()
  for (int i = 0; i < num_other; i++) {
    pos_t relative_pos = get_relative_pos(curr_pos, &other_pos[i]);
    float dist = l2_dist(&relative_pos);
    transform_to_screen(&relative_pos, offset_angle); // updates in place

    char label[16];
    sprintf(label, "%.1f m", dist);
    draw_other(&relative_pos, label);
  }
}

void update_screen(lv_display_t *disp, nmea_parser_handle_t nmea_hndl, imu_data_t *global_imu) {
    gps_output_t gps_out = read_gps(nmea_hndl);

    // Update our position if the GPS reading is valid
    // Otherwise, we just keep the last known position (from curr_pos variable).
    if (gps_out.valid) {
      curr_pos.lat = gps_out.lat;
      curr_pos.lon = gps_out.lon;
    }
    
    lora_gps_packet_t receivedValue;
    
    // Use a short timeout instead of MAX_DELAY
    const TickType_t xTicksToWait = pdMS_TO_TICKS(0);  // 0ms timeout
    if (xQueueReceive(screen_lora_event_queue, &receivedValue, xTicksToWait) == pdPASS) {
        if (receivedValue.valid == 1 && receivedValue.curr_gps_pos.lat != 0 && 
            receivedValue.curr_gps_pos.lon != 0) {
            other_pos = receivedValue.curr_gps_pos;
            ESP_LOGI("RECEIVED LAT LON", "LAT: %f, LON: %f", curr_pos.lat, curr_pos.lon);
        }
    }

    coordinates_t cory_hall = {
        CORY_DOORS_LATITUDE,
        CORY_DOORS_LONGITUDE,
    };
    
    float curr_heading = global_imu->heading;
    coordinates_t campanile_pos = {CAMPANILE_LATITUDE, CAMPANILE_LONGITUDE};
    coordinates_t positions_to_plot[2] = {
        cory_hall,
        other_pos,
    };

    // Whether device is sufficiently level
    bool is_level = fmax(fabs(global_imu->pitch), fabs(global_imu->roll)) < SCREEN_SLEEP_ORIENTATION_THRESH;

    if (!is_level) {
      gpio_set_level(2, LCD_BK_LIGHT_OFF_LEVEL);
      usleep(500 * 1000);
      return;
    } else {
      gpio_set_level(2, LCD_BK_LIGHT_ON_LEVEL);
    }
    
    ESP_LOGI("[SCREEN]", "Heading: %f", curr_heading);
    display_screen(&curr_pos, positions_to_plot, 2, curr_heading);
}

void render_counter() {

  lora_gps_packet_t receivedValue;
  if (xQueueReceive(screen_lora_event_queue, &receivedValue, portMAX_DELAY) == pdPASS) {
    pos_t send_location = {50, 0};
    pos_t recieved_location = {-50, 0};
    static char send_label[32];
    static char recieved_label[32];
    if (receivedValue.valid == 0) {
      // transmitter
      // ESP_LOGI("QUEUE READ TX", "Value from queue %ld", receivedValue.counter_val);
      sprintf(send_label, "sent: %f, %f", receivedValue.curr_gps_pos.lat, receivedValue.curr_gps_pos.lon);
    } else {
      // receiver
      // ESP_LOGI("QUEUE READ RX", "Value from queue %ld", receivedValue.counter_val);
      sprintf(recieved_label, "rec: %f, %f", receivedValue.curr_gps_pos.lat, receivedValue.curr_gps_pos.lon);

    }
    draw_bubble(&send_location, send_label);
    draw_bubble(&recieved_location, recieved_label);
  }
}

/* Clears the screen, and displays TEXT in the middle of the screen for SECONDS
    max size is ~32, I think */
void display_text(char* text) 
{
  clear_screen();
  
  pos_t origin = {0, 0};
  draw_bubble(&origin, text);
}

/* Clears the screen */
void clear_screen(void) 
{
  _lock_acquire(&lvgl_api_lock);
  lv_obj_clean(lv_screen_active());
  lv_obj_set_style_bg_color(lv_screen_active(), 
                          lv_color_hex(0x020C0E),
                          LV_PART_MAIN);
  _lock_release(&lvgl_api_lock);
}


// Note: you need to manually clear the screen for these functions to work.
void display_line_0(char* text) 
{
  pos_t line0 = {0, 75};
  draw_bubble(&line0, text);
}
void display_line_1(char* text) 
{
  pos_t line1 = {0, 25};
  draw_bubble(&line1, text);
}
void display_line_2(char* text) 
{
  pos_t line2 = {0, -25};
  draw_bubble(&line2, text);
}
void display_line_3(char* text) 
{
  pos_t line3 = {0, -75};
  draw_bubble(&line3, text);
}

/* The actual code that loops and updates the screen.
/==================================================================================================
PLS try not to clutter this example.  This should be our most up to date V0 code,
but for now it's fine to leave render_counter in here.
/==================================================================================================
 */
void screen_main_task(void *arg)
{
    screen_task_params_t* args = (screen_task_params_t*)arg;
    nmea_parser_handle_t nmea_hndl = args->nmea_hndl;
    imu_data_t* global_imu = args->global_imu;
    ESP_LOGI(SCREEN_TAG, "Starting screen_main_task");
    uint32_t time_till_next_ms = 0;
    uint32_t time_threshold_ms = 1000 / CONFIG_FREERTOS_HZ;
    while (1) {
        /* Draw the background. */
        clear_screen();
        /* Call some UI function or react on some logic ... */
        update_screen(display, nmea_hndl, global_imu);
        // render_counter(nmea_hndl);
        // varun_ui(display);
        // in case of triggering a task watch dog time out

        // Lock the mutex due to the LVGL APIs are not thread-safe
        _lock_acquire(&lvgl_api_lock);
        time_till_next_ms = lv_timer_handler();
        _lock_release(&lvgl_api_lock);
        time_till_next_ms = MAX(time_till_next_ms, time_threshold_ms);
        usleep(1000 * time_till_next_ms);
    }
    /* TODO: This should work, but for some reason it doesn't comment it out.
    Free screen_task_params that were passed to us. */
    free(arg);
}