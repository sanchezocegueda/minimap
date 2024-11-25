#include <mpu9250.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/queue.h"
#include "i2c-easy.h"
#include "esp_log.h"
#include "esp_err.h"

/* Luca: INT pin support */
#define MPU9250_RA_I2C_MST_CTRL (36)
#define MPU9250_RA_INT_ENABLE (56)
#define MPU9250_WAIT_FOR_ES_BIT (6)
#define MPU9250_RAW_RDY_EN_BIT (0)
#define MPU9250_INT_PIN (5)

static QueueHandle_t imu_irq_queue = NULL;
static void IRAM_ATTR imu_isr_handler(void* arg);

void imu_irq_init(void)
{
  /* Luca: Enable interrupts for data ready on INT pin */
  gpio_config_t io_conf = {
    .pin_bit_mask = 1ULL << MPU9250_INT_PIN,
    .mode = GPIO_MODE_INPUT,
    .pull_up_en = GPIO_PULLUP_ENABLE,
    .intr_type = GPIO_INTR_POSEDGE
  };
  gpio_config(&io_conf);

  imu_irq_queue = xQueueCreate(10, sizeof(int));

  gpio_install_isr_service(0);
  gpio_isr_handler_add(MPU9250_INT_PIN, imu_isr_handler, NULL);

  /* See MPU9250 Register Map and Descriptions section */
  // Setup Interrupt enabled register
  ESP_ERROR_CHECK(i2c_write_byte(I2C_MASTER_NUM, MPU9250_I2C_ADDR, MPU9250_RA_INT_ENABLE, 1 << MPU9250_RAW_RDY_EN_BIT));
  // We don't want to delay on the FSYNC data to be ready because we don't have any!
  ESP_ERROR_CHECK(i2c_write_bit(I2C_MASTER_NUM, MPU9250_I2C_ADDR, MPU9250_RA_I2C_MST_CTRL, MPU9250_WAIT_FOR_ES_BIT, 0));

  // Setup Interrupt Config register, 55
  /* Luca: for now, we won't check interrupt status register anyways, we will just process the isr and queue a task always */
  ESP_ERROR_CHECK(i2c_write_byte(I2C_MASTER_NUM, MPU9250_I2C_ADDR, MPU9250_RA_INT_PIN_CFG, 1 << MPU9250_INTCFG_INT_ANYRD_2CLEAR_BIT));

}

static void IRAM_ATTR imu_isr_handler(void* arg)
{
  int msg = 1;
  xQueueSendFromISR(imu_irq_queue, &msg, NULL);
}
