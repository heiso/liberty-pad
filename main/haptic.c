#include "haptic.h"
#include "driver/i2c_master.h"
#include "esp_log.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

static const char *TAG = "HAPTIC";

#define DRV2605L_ADDR 0x5A

// Registers (matching Adafruit library)
#define DRV2605_REG_STATUS 0x00
#define DRV2605_REG_MODE 0x01
#define DRV2605_REG_RTPIN 0x02
#define DRV2605_REG_LIBRARY 0x03
#define DRV2605_REG_WAVESEQ1 0x04
#define DRV2605_REG_GO 0x0C
#define DRV2605_REG_OVERDRIVE 0x0D
#define DRV2605_REG_SUSTAINPOS 0x0E
#define DRV2605_REG_SUSTAINNEG 0x0F
#define DRV2605_REG_BREAK 0x10
#define DRV2605_REG_AUDIOMAX 0x13
#define DRV2605_REG_FEEDBACK 0x1A

// I2C pins
#define I2C_SDA_GPIO 19
#define I2C_SCL_GPIO 20

static i2c_master_bus_handle_t i2c_bus;
static i2c_master_dev_handle_t drv2605_dev;
static uint8_t haptic_ready = 0;

static esp_err_t drv2605_write_reg(uint8_t reg, uint8_t val) {
  uint8_t buf[2] = { reg, val };
  return i2c_master_transmit(drv2605_dev, buf, 2, 100);
}

static esp_err_t drv2605_read_reg(uint8_t reg, uint8_t *val) {
  return i2c_master_transmit_receive(drv2605_dev, &reg, 1, val, 1, 100);
}

// Follow Adafruit library init sequence exactly
static void drv2605_configure(void) {
  // Exit standby
  drv2605_write_reg(DRV2605_REG_MODE, 0x00);

  // Disable real-time playback input
  drv2605_write_reg(DRV2605_REG_RTPIN, 0x00);

  // No overdrive, sustain, or brake
  drv2605_write_reg(DRV2605_REG_OVERDRIVE, 0);
  drv2605_write_reg(DRV2605_REG_SUSTAINPOS, 0);
  drv2605_write_reg(DRV2605_REG_SUSTAINNEG, 0);
  drv2605_write_reg(DRV2605_REG_BREAK, 0);
  drv2605_write_reg(DRV2605_REG_AUDIOMAX, 0x64);

  // Enable LRA mode: set bit 7 of feedback register
  uint8_t feedback = 0;
  drv2605_read_reg(DRV2605_REG_FEEDBACK, &feedback);
  drv2605_write_reg(DRV2605_REG_FEEDBACK, feedback | 0x80);

  // Select LRA library
  drv2605_write_reg(DRV2605_REG_LIBRARY, 6);
}

esp_err_t haptic_init(void) {
  i2c_master_bus_config_t bus_config = {
    .i2c_port = I2C_NUM_0,
    .sda_io_num = I2C_SDA_GPIO,
    .scl_io_num = I2C_SCL_GPIO,
    .clk_source = I2C_CLK_SRC_DEFAULT,
    .glitch_ignore_cnt = 7,
    .flags.enable_internal_pullup = true,
  };
  esp_err_t ret = i2c_new_master_bus(&bus_config, &i2c_bus);
  if (ret != ESP_OK) {
    ESP_LOGE(TAG, "Failed to create I2C bus: %s", esp_err_to_name(ret));
    return ret;
  }

  i2c_device_config_t dev_config = {
    .dev_addr_length = I2C_ADDR_BIT_LEN_7,
    .device_address = DRV2605L_ADDR,
    .scl_speed_hz = 400000,
  };
  ret = i2c_master_bus_add_device(i2c_bus, &dev_config, &drv2605_dev);
  if (ret != ESP_OK) {
    ESP_LOGE(TAG, "Failed to add DRV2605L device: %s", esp_err_to_name(ret));
    return ret;
  }

  // Verify device is present
  uint8_t status = 0;
  ret = drv2605_read_reg(DRV2605_REG_STATUS, &status);
  if (ret != ESP_OK) {
    ESP_LOGE(TAG, "DRV2605L not found at 0x%02x", DRV2605L_ADDR);
    return ret;
  }

  drv2605_configure();

  haptic_ready = 1;
  ESP_LOGI(TAG, "DRV2605L initialized");

  // Startup sequence
  drv2605_write_reg(DRV2605_REG_WAVESEQ1 + 0, 56);    // Pulsing Sharp 1 - 100%
  drv2605_write_reg(DRV2605_REG_WAVESEQ1 + 1, 0);     // End
  drv2605_write_reg(DRV2605_REG_GO, 1);

  return ESP_OK;
}

void haptic_click(void) {
  if (!haptic_ready) return;

  // Re-configure after standby (device loses state)
  drv2605_configure();

  // Strong Click - 100%
  drv2605_write_reg(DRV2605_REG_WAVESEQ1, 1);
  drv2605_write_reg(DRV2605_REG_WAVESEQ1 + 1, 0);
  drv2605_write_reg(DRV2605_REG_GO, 1);
}
