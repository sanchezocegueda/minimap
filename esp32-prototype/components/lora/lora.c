#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/queue.h"
#include "esp_system.h"
#include "driver/spi_master.h"
#include "soc/gpio_struct.h"
#include "driver/gpio.h"
#include <stdint.h>
#include <string.h>

#include "esp_log.h"

/*
 * Register definitions
 */
#define REG_FIFO                       0x00
#define REG_OP_MODE                    0x01
#define REG_FRF_MSB                    0x06
#define REG_FRF_MID                    0x07
#define REG_FRF_LSB                    0x08
#define REG_PA_CONFIG                  0x09
#define REG_LNA                        0x0c
#define REG_FIFO_ADDR_PTR              0x0d
#define REG_FIFO_TX_BASE_ADDR          0x0e
#define REG_FIFO_RX_BASE_ADDR          0x0f
#define REG_FIFO_RX_CURRENT_ADDR       0x10
#define REG_IRQ_FLAGS                  0x12
#define REG_RX_NB_BYTES                0x13
#define REG_PKT_SNR_VALUE              0x19
#define REG_PKT_RSSI_VALUE             0x1a
#define REG_MODEM_CONFIG_1             0x1d
#define REG_MODEM_CONFIG_2             0x1e
#define REG_PREAMBLE_MSB               0x20
#define REG_PREAMBLE_LSB               0x21
#define REG_PAYLOAD_LENGTH             0x22
#define REG_MODEM_CONFIG_3             0x26
#define REG_RSSI_WIDEBAND              0x2c
#define REG_DETECTION_OPTIMIZE         0x31
#define REG_DETECTION_THRESHOLD        0x37
#define REG_SYNC_WORD                  0x39
#define REG_DIO_MAPPING_1              0x40
#define REG_VERSION                    0x42

/*
 * Transceiver modes
 */
#define MODE_LONG_RANGE_MODE           0x80
#define MODE_SLEEP                     0x00
#define MODE_STDBY                     0x01
#define MODE_TX                        0x03
#define MODE_RX_CONTINUOUS             0x05
#define MODE_RX_SINGLE                 0x06

/*
 * PA configuration
 */
#define PA_BOOST                       0x80

/*
 * IRQ masks
 */
#define IRQ_RX_DONE_MASK               0x40

#define PA_OUTPUT_RFO_PIN              0
#define PA_OUTPUT_PA_BOOST_PIN         1

#define TIMEOUT_RESET                  100

static spi_device_handle_t __spi;

static int __implicit;
static long __frequency;

/*
 * Support for interrupts on DIO0
 */
#define DIO0_TX_DONE (1 << 6)
#define DIO0_RX_DONE 0
#define ESP_INTR_FLAG_DEFAULT 0
#define IRQ_FLAGS_RX_TIMEOUT        (1ULL << 7)
#define IRQ_FLAGS_RX_DONE           (1ULL << 6)
#define IRQ_FLAGS_PAYLOAD_CRC_ERROR (1ULL << 5)
#define IRQ_FLAGS_TX_DONE           (1ULL << 3)

/*
 * Before transmitting, we config DIO0 to interrupt TX_DONE, and config to RX_DONE before receiving.
 */
static bool __DIO0_RX = false;

/* 
 * You could also pass this as a parameter to lora_init if you wanted to define the queue elsewhere.
 */
static QueueHandle_t lora_irq_queue = NULL;

/*
 * Sleep on the lora_irq_queue and send the mask according to __DIO_RX
 */
static void IRAM_ATTR lora_isr_handler(void* arg);

/*
 * Configure IRQ_PIN for interrupts and attach lora_isr_handler.
 */
static void lora_init_irq(gpio_num_t irq_pin);

int lora_read_fifo(uint8_t *buf, int size);
void lora_write_fifo(uint8_t *buf, int size);

/**
 * Write a value to a register.
 * @param reg Register index.
 * @param val Value to write.
 */
void 
lora_write_reg(int reg, int val)
{
   uint8_t out[2] = { 0x80 | reg, val };
   uint8_t in[2];

   spi_transaction_t t = {
      .flags = 0,
      .length = 8 * sizeof(out),
      .tx_buffer = out,
      .rx_buffer = in  
   };

   gpio_set_level(CONFIG_LORA_CS_GPIO, 0);
   spi_device_transmit(__spi, &t);
   gpio_set_level(CONFIG_LORA_CS_GPIO, 1);
}

/**
 * Read the current value of a register.
 * @param reg Register index.
 * @return Value of the register.
 */
int
lora_read_reg(int reg)
{
   uint8_t out[2] = { reg, 0xff };
   uint8_t in[2];

   spi_transaction_t t = {
      .flags = 0,
      .length = 8 * sizeof(out),
      .tx_buffer = out,
      .rx_buffer = in
   };

   gpio_set_level(CONFIG_LORA_CS_GPIO, 0);
   spi_device_transmit(__spi, &t);
   gpio_set_level(CONFIG_LORA_CS_GPIO, 1);
   return in[1];
}


/**
 * Configure explicit header mode.
 * Packet size will be included in the frame.
 */
void 
lora_explicit_header_mode(void)
{
   __implicit = 0;
   lora_write_reg(REG_MODEM_CONFIG_1, lora_read_reg(REG_MODEM_CONFIG_1) & 0xfe);
}

/**
 * Configure implicit header mode.
 * All packets will have a predefined size.
 * @param size Size of the packets.
 */
void 
lora_implicit_header_mode(int size)
{
   __implicit = 1;
   lora_write_reg(REG_MODEM_CONFIG_1, lora_read_reg(REG_MODEM_CONFIG_1) | 0x01);
   lora_write_reg(REG_PAYLOAD_LENGTH, size);
}

/**
 * Sets the radio transceiver in idle mode.
 * Must be used to change registers and access the FIFO.
 */
void 
lora_idle(void)
{
   lora_write_reg(REG_OP_MODE, MODE_LONG_RANGE_MODE | MODE_STDBY);
}

/**
 * Sets the radio transceiver in sleep mode.
 * Low power consumption and FIFO is lost.
 */
void 
lora_sleep(void)
{ 
   lora_write_reg(REG_OP_MODE, MODE_LONG_RANGE_MODE | MODE_SLEEP);
}

/**
 * Sets the radio transceiver in receive mode.
 * Incoming packets will be received.
 */
void 
lora_receive(void)
{
   lora_write_reg(REG_OP_MODE, MODE_LONG_RANGE_MODE | MODE_RX_CONTINUOUS);
}

/**
 * Configure power level for transmission
 * @param level 2-17, from least to most power
 */
void 
lora_set_tx_power(int level)
{
   // RF9x module uses PA_BOOST pin
   if (level < 2) level = 2;
   else if (level > 17) level = 17;
   lora_write_reg(REG_PA_CONFIG, PA_BOOST | (level - 2));
}

/**
 * Set carrier frequency.
 * @param frequency Frequency in Hz
 */
void 
lora_set_frequency(long frequency)
{
   __frequency = frequency;

   uint64_t frf = ((uint64_t)frequency << 19) / 32000000;

   lora_write_reg(REG_FRF_MSB, (uint8_t)(frf >> 16));
   lora_write_reg(REG_FRF_MID, (uint8_t)(frf >> 8));
   lora_write_reg(REG_FRF_LSB, (uint8_t)(frf >> 0));
}

/**
 * Set spreading factor.
 * @param sf 6-12, Spreading factor to use.
 */
void 
lora_set_spreading_factor(int sf)
{
   if (sf < 6) sf = 6;
   else if (sf > 12) sf = 12;

   if (sf == 6) {
      lora_write_reg(REG_DETECTION_OPTIMIZE, 0xc5);
      lora_write_reg(REG_DETECTION_THRESHOLD, 0x0c);
   } else {
      lora_write_reg(REG_DETECTION_OPTIMIZE, 0xc3);
      lora_write_reg(REG_DETECTION_THRESHOLD, 0x0a);
   }

   lora_write_reg(REG_MODEM_CONFIG_2, (lora_read_reg(REG_MODEM_CONFIG_2) & 0x0f) | ((sf << 4) & 0xf0));
}

/**
 * Set bandwidth (bit rate)
 * @param sbw Bandwidth in Hz (up to 500000)
 */
void 
lora_set_bandwidth(long sbw)
{
   int bw;

   if (sbw <= 7.8E3) bw = 0;
   else if (sbw <= 10.4E3) bw = 1;
   else if (sbw <= 15.6E3) bw = 2;
   else if (sbw <= 20.8E3) bw = 3;
   else if (sbw <= 31.25E3) bw = 4;
   else if (sbw <= 41.7E3) bw = 5;
   else if (sbw <= 62.5E3) bw = 6;
   else if (sbw <= 125E3) bw = 7;
   else if (sbw <= 250E3) bw = 8;
   else bw = 9;
   lora_write_reg(REG_MODEM_CONFIG_1, (lora_read_reg(REG_MODEM_CONFIG_1) & 0x0f) | (bw << 4));
}

/**
 * Set coding rate 
 * @param denominator 5-8, Denominator for the coding rate 4/x
 */ 
void 
lora_set_coding_rate(int denominator)
{
   if (denominator < 5) denominator = 5;
   else if (denominator > 8) denominator = 8;

   int cr = denominator - 4;
   lora_write_reg(REG_MODEM_CONFIG_1, (lora_read_reg(REG_MODEM_CONFIG_1) & 0xf1) | (cr << 1));
}

/**
 * Set the size of preamble.
 * @param length Preamble length in symbols.
 */
void 
lora_set_preamble_length(long length)
{
   lora_write_reg(REG_PREAMBLE_MSB, (uint8_t)(length >> 8));
   lora_write_reg(REG_PREAMBLE_LSB, (uint8_t)(length >> 0));
}

/**
 * Change radio sync word.
 * @param sw New sync word to use.
 */
void 
lora_set_sync_word(int sw)
{
   lora_write_reg(REG_SYNC_WORD, sw);
}

/**
 * Enable appending/verifying packet CRC.
 */
void 
lora_enable_crc(void)
{
   lora_write_reg(REG_MODEM_CONFIG_2, lora_read_reg(REG_MODEM_CONFIG_2) | 0x04);
}

/**
 * Disable appending/verifying packet CRC.
 */
void 
lora_disable_crc(void)
{
   lora_write_reg(REG_MODEM_CONFIG_2, lora_read_reg(REG_MODEM_CONFIG_2) & 0xfb);
}

/**
 * Perform hardware initialization.
 */
int 
lora_init(void)
{
   esp_err_t ret;

   /*
    * Configure CPU hardware to communicate with the radio chip
    */
   esp_rom_gpio_pad_select_gpio(CONFIG_LORA_CS_GPIO);
   gpio_set_direction(CONFIG_LORA_CS_GPIO, GPIO_MODE_OUTPUT);

   if (CONFIG_LORA_IRQ_ENABLED)
      lora_init_irq(CONFIG_LORA_DIO0_GPIO);

   spi_bus_config_t bus = {
      .miso_io_num = CONFIG_LORA_MISO_GPIO,
      .mosi_io_num = CONFIG_LORA_MOSI_GPIO,
      .sclk_io_num = CONFIG_LORA_SCK_GPIO,
      .quadwp_io_num = -1,
      .quadhd_io_num = -1,
      .max_transfer_sz = 0
   };
           
   ret = spi_bus_initialize(VSPI_HOST, &bus, 0);
   assert(ret == ESP_OK);

   spi_device_interface_config_t dev = {
      .clock_speed_hz = 9000000,
      .mode = 0,
      .spics_io_num = -1,
      .queue_size = 1,
      .flags = 0,
      .pre_cb = NULL
   };
   ret = spi_bus_add_device(VSPI_HOST, &dev, &__spi);
   assert(ret == ESP_OK);


   /*
    * Check version.
    */
   uint8_t version;
   uint8_t i = 0;
   while(i++ < TIMEOUT_RESET) {
      version = lora_read_reg(REG_VERSION);
      if(version == 0x12) break;
      vTaskDelay(2);
   }
   assert(i <= TIMEOUT_RESET + 1); // at the end of the loop above, the max value i can reach is TIMEOUT_RESET + 1

   /*
    * Default configuration.
    */
   lora_sleep();
   lora_write_reg(REG_FIFO_RX_BASE_ADDR, 0);
   lora_write_reg(REG_FIFO_TX_BASE_ADDR, 0);
   lora_write_reg(REG_LNA, lora_read_reg(REG_LNA) | 0x03);
   lora_write_reg(REG_MODEM_CONFIG_3, 0x04);
   lora_set_tx_power(17);

   lora_idle();
   return 1;
}

/**
 * Send a packet.
 * @param buf Data to be sent
 * @param size Size of data.
 */
void 
lora_send_packet(uint8_t *buf, int size)
{
   lora_write_fifo(buf, size);
   /*
    * Start transmission and wait for conclusion.
    */
   lora_write_reg(REG_OP_MODE, MODE_LONG_RANGE_MODE | MODE_TX);
   while((lora_read_reg(REG_IRQ_FLAGS) & IRQ_FLAGS_TX_DONE) == 0)
      vTaskDelay(2);

   lora_write_reg(REG_IRQ_FLAGS, IRQ_FLAGS_TX_DONE);
}

/**
 * Puts the radio in transmit mode then blocks on lora_irq_queue until a message is done transmitting.
 * @param buf Data to be sent
 * @param size Size of data.
 */
void
lora_send_packet_blocking(uint8_t *buf, int size)
{
   lora_write_fifo(buf, size);
   lora_write_reg(REG_DIO_MAPPING_1, DIO0_TX_DONE); /* Config DIO0 to interrupt when TxDone */
   __DIO0_RX = false;

   /*
    * Start transmission and wait for conclusion.
    */
   lora_write_reg(REG_OP_MODE, MODE_LONG_RANGE_MODE | MODE_TX);
   int event;
   // TODO: Make 5000 a customizable delay
   if (xQueueReceive(lora_irq_queue, &event, 5000) && event == IRQ_FLAGS_TX_DONE)
         lora_write_reg(REG_IRQ_FLAGS, IRQ_FLAGS_TX_DONE); // clear bit in IRQ Status register
}

int lora_read_fifo(uint8_t *buf, int size)
{
   int len = 0;
   /*
    * Find packet size.
    */
   if (__implicit) len = lora_read_reg(REG_PAYLOAD_LENGTH);
   else len = lora_read_reg(REG_RX_NB_BYTES);

   /*
    * Transfer data from radio.
    */
   lora_idle();   
   lora_write_reg(REG_FIFO_ADDR_PTR, lora_read_reg(REG_FIFO_RX_CURRENT_ADDR));
   if(len > size) len = size;
   for(int i=0; i<len; i++) 
      *buf++ = lora_read_reg(REG_FIFO);

   return len;
}

void lora_write_fifo(uint8_t *buf, int size)
{
   /*
    * Transfer data to radio.
    */
   lora_idle();
   lora_write_reg(REG_FIFO_ADDR_PTR, 0);

   for(int i=0; i<size; i++) 
      lora_write_reg(REG_FIFO, *buf++);
   
   lora_write_reg(REG_PAYLOAD_LENGTH, size);
}

/**
 * Read a received packet.
 * @param buf Buffer for the data.
 * @param size Available size in buffer (bytes).
 * @return Number of bytes received (zero if no packet available).
 */
int 
lora_receive_packet(uint8_t *buf, int size)
{
   /*
    * Check interrupts.
    */
   int irq = lora_read_reg(REG_IRQ_FLAGS);
   lora_write_reg(REG_IRQ_FLAGS, irq);
   if((irq & IRQ_RX_DONE_MASK) == 0) return 0;
   if(irq & IRQ_FLAGS_PAYLOAD_CRC_ERROR) return 0;

   return lora_read_fifo(buf, size);
}

/**
 * Returns non-zero if there is data to read (packet received).
 */
int
lora_received(void)
{
   if(lora_read_reg(REG_IRQ_FLAGS) & IRQ_RX_DONE_MASK) return 1;
   return 0;
}

/**
 * Puts the radio in receive mode then blocks on lora_irq_queue until a packet is received.
 * @param buf Buffer for the data.
 * @param size Available size in buffer (bytes).
 * @return Number of bytes received (zero if no packet available).
 */
int
lora_receive_packet_blocking(uint8_t *buf, int size)
{
   lora_write_reg(REG_DIO_MAPPING_1, DIO0_RX_DONE); /* Config DIO0 to interrupt when RxDone */
   __DIO0_RX = true;
   lora_receive();
   int event;
    if (xQueueReceive(lora_irq_queue, &event, portMAX_DELAY) && event == IRQ_FLAGS_RX_DONE) {
      int irq = lora_read_reg(REG_IRQ_FLAGS);
      if(irq & IRQ_FLAGS_PAYLOAD_CRC_ERROR) {
         ESP_LOGI("[DEBUG LORA BLOCKING]", "CRC error");
         lora_write_reg(REG_IRQ_FLAGS, IRQ_FLAGS_PAYLOAD_CRC_ERROR | IRQ_FLAGS_RX_DONE); // clear bit in IRQ Status register
         return 0;
      }
      if (irq & IRQ_FLAGS_RX_TIMEOUT) {
         ESP_LOGI("[DEBUG LORA BLOCKING]", "timeout");
         lora_write_reg(REG_IRQ_FLAGS, IRQ_FLAGS_RX_TIMEOUT | IRQ_FLAGS_RX_DONE); // clear bit in IRQ Status register
         return 0;
      }
      ESP_LOGI("[DEBUG LORA BLOCKING]", "No errors");
      lora_write_reg(REG_IRQ_FLAGS, IRQ_FLAGS_RX_DONE); // clear bit in IRQ Status register
      return lora_read_fifo(buf, size);
    }
   return 0;
}

/**
 * Return last packet's RSSI.
 */
int 
lora_packet_rssi(void)
{
   return (lora_read_reg(REG_PKT_RSSI_VALUE) - (__frequency < 868E6 ? 164 : 157));
}

/**
 * Return last packet's SNR (signal to noise ratio).
 */
float 
lora_packet_snr(void)
{
   return ((int8_t)lora_read_reg(REG_PKT_SNR_VALUE)) * 0.25;
}

/**
 * Shutdown hardware.
 */
void 
lora_close(void)
{
   lora_sleep();
//   close(__spi);  FIXME: end hardware features after lora_close
//   close(__cs);
//   close(__rst);
//   __spi = -1;
//   __cs = -1;
//   __rst = -1;
}

void 
lora_dump_registers(void)
{
   int i;
   printf("00 01 02 03 04 05 06 07 08 09 0A 0B 0C 0D 0E 0F\n");
   for(i=0; i<0x40; i++) {
      printf("%02X ", lora_read_reg(i));
      if((i & 0x0f) == 0x0f) printf("\n");
   }
   printf("\n");
}

/**
 * ISR for DIO0 interrupt; send the proper event based on __DIO0_RX.
 */
static void IRAM_ATTR
lora_isr_handler(void* arg)
{
   int event = __DIO0_RX ? IRQ_FLAGS_RX_DONE : IRQ_FLAGS_TX_DONE;
   xQueueSendFromISR(lora_irq_queue, &event, NULL);
}

/**
 * Configure IRQ_PIN as DIO0 rising edge interrupt, install lora_isr_handler,
 * and create the lora_irq_queue for processing interrupts.
 */
static void
lora_init_irq(gpio_num_t irq_pin)
{
   gpio_config_t io_conf = {
      .pin_bit_mask = 1ULL << irq_pin,
      .mode = GPIO_MODE_INPUT,
      .pull_up_en = GPIO_PULLDOWN_ENABLE,
      .intr_type = GPIO_INTR_POSEDGE
   };
   gpio_config(&io_conf);

   lora_irq_queue = xQueueCreate(10, sizeof(int));
   gpio_install_isr_service(ESP_INTR_FLAG_DEFAULT);
   gpio_isr_handler_add(irq_pin, lora_isr_handler, NULL);
}
