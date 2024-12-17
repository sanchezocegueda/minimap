#include <stdio.h>
#include <string.h>
#include <stdlib.h>

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/queue.h"

#include "esp_log.h"
#include "esp_system.h"
#include "esp_err.h"
#include "esp_task_wdt.h"

#include "driver/i2c.h"

#include "nmea_parser.h"

#include "ui_utils.h"

/* Code for our 2 physical buttons */
#include "buttons.h"

/* IMU */
#include "ahrs.h"
#include "mpu9250.h"
#include "calibrate.h"
#include "common.h"

#include "lora.h"

#include "psa/crypto.h"

/* TODO: Cleanup */
void send_lora_gps();

#define I2C_MASTER_NUM I2C_NUM_0 /* I2C port number for master dev */

const bool DEBUG_IMU = true;


/* Global variable init */
imu_data_t global_imu;
nmea_parser_handle_t nmea_hndl;
QueueHandle_t button_event_queue;
QueueHandle_t screen_lora_event_queue;

coordinates_t curr_pos;
coordinates_t other_pos;

/* IMU Calibration, set accordingly to hardware in the cal variable below. */
calibration_t cal_mpu92_65 = {
    .mag_offset = {.x = 44.384766, .y = -12.468750, .z = 24.607422},
    .mag_scale = {.x = 1.023329, .y = 1.019963, .z = 0.959353},
    .gyro_bias_offset = {.x = -2.226222, .y = -1.335986, .z = -2.275180},
    .accel_offset = {.x = -0.010997, .y = 0.026934, .z = 0.042839},
    .accel_scale_lo = {.x = 1.000212, .y = 1.015479, .z = 1.029726},
    .accel_scale_hi = {.x = -1.001975, .y = -0.988439, .z = -0.996091}};

calibration_t cal_mpu9250_6500 = {
    .mag_offset = {.x = 27.246094, .y = 28.031250, .z = 19.335938},
    .mag_scale = {.x = 0.971636, .y = 0.923795, .z = 1.125724},
    .gyro_bias_offset = {.x = -0.612455, .y = -0.792115, .z = -1.854843},
    .accel_offset = {.x = 0.028677, .y = 0.059277, .z = 0.060068},
    .accel_scale_lo = {.x = 1.016573, .y = 1.025387, .z = 1.041263},
    .accel_scale_hi = {.x = -0.984251, .y = -0.977627, .z = -0.985206}};

/* Global calibration variable */
calibration_t *cal = &cal_mpu9250_6500;


/* Function to run the imu */
static void run_imu(void)
{
  uint32_t i = 0;
  while (true)
  {
    vector_t va, vg, vm;

    /* Get the raw Accelerometer, Gyroscope and Magnetometer values. */
    ESP_ERROR_CHECK(get_accel_gyro_mag(&va, &vg, &vm));

    /* Transform these to the orientation of our device. */
    transform_accel_gyro(&va);
    transform_accel_gyro(&vg);
    // transform_mag(&vm); Not needed because we keep the axis the same

    /* Apply the AHRS algorithm */
    ahrs_update(DEG2RAD(vg.x), DEG2RAD(vg.y), DEG2RAD(vg.z),
                va.x, va.y, va.z,
                vm.x, vm.y, vm.z);

    /* Update global_imu data */
    float heading, pitch, roll;
    ahrs_get_euler_in_degrees(&heading, &pitch, &roll);
    // TODO: Add semaphore
    global_imu.heading = heading;
    global_imu.pitch = pitch;
    global_imu.roll = roll;

    /* Debug every CONFIG_SAMPLE_RATE_Hz / 10 */
    if (DEBUG_IMU && i++ % 100 == 0) {
      ESP_LOGI("[IMU]", "heading: %2.3f°, pitch: %2.3f°, roll: %2.3f°", heading, pitch, roll);
    }
      // TODO: Shouldn't need this below
      // vTaskDelay(1); /* Make the WDT happy */

    imu_pause(); /* Delay imu task to reach target sample frequency, set in menuconfig under CONFIG_SAMPLE_RATE_Hz */
  }
}


static void imu_task(void *arg)
{
  /* Init IMU and ahrs algorithm */
  assert(ESP_OK == init_imu(cal));
  ahrs_init(SAMPLE_FREQ_Hz, 0.8);

  run_imu(); /* Infinite loop to poll the IMU in a period of CONFIG_SAMPLE_RATE_Hz */

  /* Cleanup task */
  vTaskDelay(100 / portTICK_PERIOD_MS);
  i2c_driver_delete(I2C_MASTER_NUM);
  vTaskDelete(NULL);
}


/* Basic transmit task to send a counter every 2.5 seconds */
void task_tx(void *p)
{
  ESP_LOGI("[TX]", "started");
  lora_packet_t packet_tx = {
    .valid = 0, // indicate that this is transmitter
    .counter_val = 0,
  };
  for (;;)
  {
    lora_send_packet((uint8_t *)&packet_tx.counter_val, sizeof(uint32_t));
    ESP_LOGI("[TX]", "counter: %ld", packet_tx.counter_val);
    xQueueSendToFront(screen_lora_event_queue, &packet_tx, portMAX_DELAY);
    packet_tx.counter_val++;
    vTaskDelay(pdMS_TO_TICKS(500)); // Send a tx every 500 ms
  }
}


/* Basic receive task to turn radio into receive mode and block until data is received */
void task_rx(void *p)
{
  ESP_LOGI("[RX]", "started");
  uint8_t buf[sizeof(uint32_t)];
  lora_packet_t packet_rx = {
    .counter_val = 0,
    .valid = 1, // indicate that this is receiver
  };

  for (;;)
  {
    lora_receive(false);
    bool received;
    while (!(received = lora_received())) { vTaskDelay(pdMS_TO_TICKS(25)); }

    if (received) {
      int bytes_read = lora_receive_packet(buf, sizeof(uint32_t));
      if (bytes_read > 0) {
        ESP_LOGI("[RX]", "%d bytes, counter: %ld", bytes_read, (uint32_t)buf[0]);
        packet_rx.counter_val = (uint32_t)buf[0];
        xQueueSend(screen_lora_event_queue, &packet_rx, portMAX_DELAY);
      }
    }

    vTaskDelay(1);
  }
}


/* Task-like function to send a counter and listen for a counter periodically. */
void task_both(nmea_parser_handle_t nmea_hndl)
{
  uint8_t buf[sizeof(coordinates_t)];

   /* Set the size of packets to match GPS data. (By default it is configured to uint32_t in lora_config()) */
   lora_implicit_header_mode(sizeof(coordinates_t));

  for (;;)
  {

    gps_time_t time = read_gps_time(nmea_hndl);
    

    // > 5 is for 9250-6500
    // < 5 is 92-65
    if (time.second % 10 > 5) { // TODO: CHANGE THIS TO > 5 for other device (need to find cleaner way so we don't have to change)
      gps_output_t gps_out = read_gps(nmea_hndl);

      // Make sure we have a valid GPS reading
      // Otherwise just try again
      if (!gps_out.valid) {
        continue;
      }

      curr_pos.lat = gps_out.lat;
      curr_pos.lon = gps_out.lon;
      
      lora_send_packet_blocking((uint8_t *) &curr_pos, sizeof(coordinates_t));
      ESP_LOGI("SENT LAT LON", "LAT: %f, LON: %f", curr_pos.lat, curr_pos.lon);
    } else {
      // lora_receive(false);
      gps_output_t gps_rx;
      /* Make this some small interval */
      // while (lora_received()) {
        int bytes_read = lora_receive_packet_blocking((uint8_t *)&buf, sizeof(coordinates_t));
        if (bytes_read > 0) {
          other_pos = ((coordinates_t*)buf)[0];
          gps_rx.lat = other_pos.lat;
          gps_rx.lon = other_pos.lon;
          xQueueSendToFront(screen_lora_event_queue, &other_pos, portMAX_DELAY);
        }
      // }
    }  
  }
}

/* Task to send a counter and listen for a counter periodically */
void task_both_gps(void *p)
{
  ESP_LOGI("[task both gps]", "started");

  /* Change payload length to match GPS data */
  lora_implicit_header_mode(sizeof(coordinates_t));

  coordinates_t buf;
  int bytes_read;
  for (;;)
  {
    send_lora_gps(nmea_hndl);

    // lora_receive(false);
    // bool received;
    // while (!(received = lora_received())) { vTaskDelay(pdMS_TO_TICKS(25)); }

    // if (received) {
      int bytes_read = lora_receive_packet_blocking((uint8_t *)&buf, sizeof(coordinates_t));
      if (bytes_read > 0) {
        ESP_LOGI("[LORA_RX_GPS]", "Bytes read: %d, Lat: %0.5f, Long: %0.5f", bytes_read, buf.lat, buf.lon);
        other_pos = buf;
      }
    // }
    vTaskDelay(1);
  }
}

/**
 * @brief Format latitude, longitude and send via lora
 * @param nmea_hndl returned from nmea_parser_init
 * */
void send_lora_gps(nmea_parser_handle_t nmea_hndl)
{
  gps_output_t gps_out = read_gps(nmea_hndl);
  
  if (!gps_out.valid) {
    return;
  }

  // Update our current position
  curr_pos.lat = gps_out.lat;
  curr_pos.lon = gps_out.lon;
  // TODO: encryption

  lora_send_packet_blocking((uint8_t *)&curr_pos, sizeof(coordinates_t));
  ESP_LOGI("[LORA_TX_GPS]", "Lat: %0.5f, Long: %0.5f", curr_pos.lat, curr_pos.lon);
}

/* Task for receiving GPS over lora */
void receive_lora_gps(void *)
{
  ESP_LOGI("[LORA_RX_GPS]", "started");

  /* Change payload length to match GPS data */
  lora_implicit_header_mode(sizeof(coordinates_t));

  coordinates_t buf;
  for (;;)
  {
    // bool received;
    // while (!(received = lora_received())) { vTaskDelay(pdMS_TO_TICKS(25)); }

    // if (received) {
      int bytes_read = lora_receive_packet_blocking((uint8_t *)&buf, sizeof(coordinates_t));
      if (bytes_read > 0) {
        ESP_LOGI("[LORA_RX_GPS]", "Bytes read: %d, Lat: %0.5f, Long: %0.5f", bytes_read, buf.lat, buf.lon);
      }
    // }
    vTaskDelay(1);
  }
}


// TODO: we could also include the 'ID' of the minimap device, i.e. which interval it's in
typedef struct lora_task_params {
    nmea_parser_handle_t nmea_hndl;
} lora_task_params_t;

/* Main RTOS Lora task that does hardware initialization. */
void lora_task(void *pvParam)
{
  lora_task_params_t * params = (lora_task_params_t *) pvParam;
  /* Setup hardware for Lora Radio, initialize SPI driver */
  assert(1 == lora_init());
  lora_config(); /* Configure hardware registers for application */

  /* Start an actual task for the radio... */
  // task_rx(NULL);
  // task_tx(NULL);
  // task_both_gps(NULL);
  task_both(params->nmea_hndl);
  // task_lora_rx(NULL);
}

/* Start of Minimap */
void app_main()
{
  /* Setup GPS */
  nmea_parser_config_t config = NMEA_PARSER_CONFIG_DEFAULT();
  nmea_hndl = nmea_parser_init(&config);

  /* Setup buttons */
  button_handle_t left_btn, right_btn;
  init_buttons(&left_btn, &right_btn);

  /* Register default debugging callback functions. */
  iot_button_register_cb(left_btn, BUTTON_PRESS_DOWN, left_cb, NULL);
  iot_button_register_cb(right_btn, BUTTON_PRESS_DOWN, right_cb, NULL);

  /* Run calibration task. */
  calibrate_screen_params_t calibrate_params = {
    .button_event_queue = NULL, // FIXME: IGNORE for now...
    .cal_x = cal,
  };

  /* Initialize hardware for the screen, and start LVGL.*/
  start_screen();

  /* Calibrate_task returns and does not infinite loop, so it's ok to use stack memory.
  app_main runs this to completion before executing the next line of code.  */
  calibrate_task(&calibrate_params);

  /* Setup IMU and start the polling task. */
  xTaskCreate(imu_task, "imu_task", 4096, NULL, 10, NULL);

  /* Malloc screen parameters for GPS data, counter data, imu data, and pass to lora task and screen task.
  Currently, this freed when screen_main_task cleans itself up (after the infinite loop) */
  /* tbh idk if storing all this on the heap is really cleaner than just using static memory...  but, I feel like it's not that bad */
  lora_task_params_t * lora_task_params = malloc(sizeof(lora_task_params_t));
  screen_task_params_t *screen_params = malloc(sizeof(screen_task_params_t));
  // screen_params->screen_lora_event_queue = xQueueCreate(10, sizeof(struct lora_packet));
  /* TODO: Cleanup global_imu... Make a thread_safe ds similar to gps_t in nmea_parser.c */
  screen_params->global_imu = &global_imu;
  screen_params->nmea_hndl = nmea_hndl;
  lora_task_params->nmea_hndl = nmea_hndl;

  screen_lora_event_queue = xQueueCreate(20, sizeof(coordinates_t));

  xTaskCreate(&lora_task, "lora_task", 4096, lora_task_params, 5, NULL);
  
  xTaskCreate(screen_main_task, "Minimap", 8192, screen_params, LVGL_TASK_PRIORITY, NULL);
}
