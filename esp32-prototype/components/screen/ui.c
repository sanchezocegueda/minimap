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

#define CAMPANILE_LONGITUDE -122.25777  // campanile longitude (in degrees)
#define CAMPANILE_LATITUDE 37.87194     // campanile latitude (in degrees)
#define SATHERGATE_LONGITUDE -122.25947 // sather gate longitude (in degrees)
#define SATHERGATE_LATITUDE 37.8702180  // sather gate latitude (in degrees)
#define EARTH_RADIUS_M 6371000.0        // earth's radius in meters
#define MAX_DISPLAY_RADIUS 100
#define MIN_DISPLAY_RADIUS 20
#define SCREEN_SCALE 2

/* NOTE: y values are flipped on the screen. */

/* GPS Data from right outside cory doors. */
#define CORY_DOORS_LONGITUDE -122.25798
#define CORY_DOORS_LATITUDE 37.87535
#define CORY_DOORS_ALTITUDE 150.10001

/* GPS Coordinate */
typedef struct coordinates
{
    float lat;
    float lon;
} coordinates_t;

/* Euclidean Coordinate */
typedef struct pos
{
    float x;
    float y;
} pos_t;

/* Get distance from origin */
float l2_dist(pos_t *pos)
{
    return sqrt((pos->y * pos->y) + (pos->x * pos->x));
}

/* Converts angle in degrees to radians */
float deg_to_rad(float deg)
{
    return deg * (M_PI / 180.0f);
}

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
    float a = sinf(d_lat/2) * sinf(d_lat/2) +
               cosf(lat1_rad) * cosf(lat2_rad) *
               sinf(d_lon/2) * sinf(d_lon/2);

    float central_angle = 2 * atan2(sqrt(a), sqrt(1-a));

    float total_distance = EARTH_RADIUS_M * central_angle;

    /* Calculate x and y components
       x is East-West (longitude difference)
       y is North-South (latitude difference) */
    float x_dist = EARTH_RADIUS_M * (lon2_rad - lon1_rad) * 
                  cos((lat1_rad + lat2_rad) / 2);
    float y_dist = EARTH_RADIUS_M * (lat2_rad - lat1_rad);

    pos_t dist_relative = {
        x_dist,
        y_dist
    };

    return dist_relative;
}

/**
 * @brief Rotates pos by heading_angle and scales to screen.
 * @param pos Expects pos in meters. MUTATED
 * @param heading_angle expects angle in degrees
 * 
 * Converts pos to polar then add the heading angle. Then convert back to rectangular.
 */
void transform_to_screen(pos_t* pos, float heading_angle)
{
    /* Calculate rotation angle relative to heading */
    float r = sqrt((pos->x) * (pos->x) + (pos->y) * (pos->y));
    float angle1 = atan2(pos->y, pos->x);
    float angle2 = deg_to_rad(heading_angle); // TODO: Verify we didn't need the negative @Alex
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
void draw_bubble(pos_t * position, char *label)
{
    /* Initialize style for the bubble */
    static lv_style_t style_bubble;
    lv_style_init(&style_bubble);
    lv_style_set_radius(&style_bubble, 20); // Set the radius for rounded corners
    lv_style_set_bg_opa(&style_bubble, LV_OPA_COVER);
    lv_style_set_bg_color(&style_bubble, lv_color_hex(0x23CEF1));

    /* Create bubble object */
    lv_obj_t *bubble_obj = lv_obj_create(lv_scr_act()); // Add to the active screen
    lv_obj_set_size(bubble_obj, 20, 20);               // Set size of the bubble
    lv_obj_add_style(bubble_obj, &style_bubble, 0);    // Add the style to the bubble
    lv_obj_align(bubble_obj, LV_ALIGN_CENTER, position->x, position->y); // Align the bubble

    /* Create label object */
    lv_obj_t *label_obj = lv_label_create(lv_scr_act()); // Add label to the active screen
    lv_label_set_text(label_obj, label);                // Set the label text
    lv_obj_set_style_text_color(label_obj, lv_color_hex(0xFFFFFF), LV_PART_MAIN); // Set text color
    lv_obj_align_to(label_obj, bubble_obj, LV_ALIGN_OUT_BOTTOM_MID, 0, 10); // Align the label to bubble

}

/**
 * @brief Draw indicator pointing toward north
 */
void draw_north_indicator(float heading_angle) {

    float angle = deg_to_rad(-heading_angle);
    float x = cosf(angle) * MAX_DISPLAY_RADIUS;
    float y = sinf(angle) * MAX_DISPLAY_RADIUS;

    /* Initialize style for the bubble */
    static lv_style_t style_bubble;
    lv_style_init(&style_bubble);
    lv_style_set_radius(&style_bubble, 10); // Set the radius for rounded corners
    lv_style_set_bg_opa(&style_bubble, LV_OPA_COVER);
    lv_style_set_bg_color(&style_bubble, lv_color_hex(0xFF8888));

    /* Create bubble object */
    lv_obj_t *bubble_obj = lv_obj_create(lv_scr_act()); // Add to the active screen
    lv_obj_set_size(bubble_obj, 10, 10);               // Set size of the bubble
    lv_obj_add_style(bubble_obj, &style_bubble, 0);    // Add the style to the bubble
    lv_obj_align(bubble_obj, LV_ALIGN_CENTER, x, y); // Align the bubble

    /* Create label object */
    lv_obj_t *label_obj = lv_label_create(lv_scr_act()); // Add label to the active screen
    lv_label_set_text_fmt(label_obj, "%.0f °", heading_angle);                // Set the label text
    lv_obj_set_style_text_color(label_obj, lv_color_hex(0xFFFFFF), LV_PART_MAIN); // Set text color
    lv_obj_align_to(label_obj, bubble_obj, LV_ALIGN_OUT_BOTTOM_MID, 0, 10); // Align the label to bubble
}

/**
 * @brief Displays every GPS point-of-interest on the screen.
 * @param curr_pos Current user GPS position
 * @param other_pos Array of other points of interest
 * @param num_other Length of other_pos array
 * @param offset_angle Current heading relative to north
 */
void display_screen(coordinates_t *curr_pos, coordinates_t *other_pos, int num_other, float offset_angle)
{
    lv_obj_clean(lv_screen_active());
    lv_obj_set_style_bg_color(lv_screen_active(), lv_color_hex(0x020C0E), LV_PART_MAIN);
    
    pos_t origin = {0, 0};
    char our_label[32];
    sprintf(our_label, "%.5f °N\n%.5f °E", curr_pos->lat, curr_pos->lon);
    draw_bubble(&origin, our_label);

    // TODO: Appears to be 90 degrees off for some reason? Needs debugging
    draw_north_indicator(offset_angle);
    
    for (int i = 0; i < num_other; i++)
    {
        pos_t relative_pos = get_relative_pos(curr_pos, &other_pos[i]);
        float dist = l2_dist(&relative_pos);
        transform_to_screen(&relative_pos, offset_angle); //updates in place
        
        char label[16];
        sprintf(label, "%.1f m", dist);
        draw_bubble(&relative_pos, label);
    }
}

/* Update information for the screen and display it. Gets called every 2ms. */
void update_screen(lv_display_t *disp, gps_t *global_gps, imu_data_t *global_imu)
{
    // TODO: unjank
    if (global_gps->latitude == 0.0 || global_gps->longitude == 0.0) {
        return;
    }

    coordinates_t curr_pos = {
        global_gps->latitude,
        global_gps->longitude
    };

    coordinates_t cory_hall = {
        CORY_DOORS_LATITUDE,
        CORY_DOORS_LONGITUDE,
    };

    float curr_heading = global_imu->heading;

    coordinates_t campanile_pos = {
        CAMPANILE_LATITUDE,
        CAMPANILE_LONGITUDE
    };
    
    coordinates_t positions_to_plot[1] = {
        cory_hall,
    };

    display_screen(&curr_pos, positions_to_plot, 1, curr_heading);
}
