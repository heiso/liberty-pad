#include "sensor.h"
#include "esp_log.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "main.h"
#include "sdkconfig.h"
#include <string.h>

static const char *TAG = "SENSOR";

#define CONVERSION_FRAME_SIZE (SOC_ADC_DIGI_DATA_BYTES_PER_CONV * ADC_CHANNEL_COUNT)
#define CONVERSION_POOL_SIZE CONVERSION_FRAME_SIZE * 1

extern const uint32_t adc_channels[ADC_CHANNEL_COUNT];

adc_continuous_handle_t adc_handle;
static TaskHandle_t adc_task_handle;

extern void update_key_state(adc_channel_t adc_channel, uint16_t raw_value);
extern void update_battery_voltage(uint16_t raw_value);

static bool IRAM_ATTR
on_conversion_done_cb(adc_continuous_handle_t handle,
                      const adc_continuous_evt_data_t *edata, void *user_data) {
  BaseType_t mustYield = pdFALSE;
  // Notify that ADC continuous driver has done enough number of conversions
  vTaskNotifyGiveFromISR(adc_task_handle, &mustYield);

  return (mustYield == pdTRUE);
}

void adc_init() {
  //-------------ADC Init---------------//
  adc_continuous_handle_cfg_t adc_config = {
    .max_store_buf_size = CONVERSION_POOL_SIZE,
    .conv_frame_size = CONVERSION_FRAME_SIZE,
    .flags = { .flush_pool = 1 }
  };
  ESP_ERROR_CHECK(adc_continuous_new_handle(&adc_config, &adc_handle));

  //-------------ADC Config---------------//
  adc_continuous_config_t config = {
    .pattern_num = ADC_CHANNEL_COUNT,
    .sample_freq_hz = SOC_ADC_SAMPLE_FREQ_THRES_HIGH,
    .conv_mode = ADC_CONV_SINGLE_UNIT_1,
    .format = ADC_DIGI_OUTPUT_FORMAT_TYPE2,
  };

  adc_digi_pattern_config_t adc_pattern[ADC_CHANNEL_COUNT] = { 0 };
  for (int i = 0; i < ADC_CHANNEL_COUNT; i++) {
    adc_pattern[i].atten = ADC_ATTEN_DB_12;
    adc_pattern[i].channel = adc_channels[i];
    adc_pattern[i].unit = ADC_UNIT_1;
    adc_pattern[i].bit_width = SOC_ADC_DIGI_MAX_BITWIDTH;

    ESP_LOGI(TAG, "adc_pattern[%d].channel is :%" PRIx8, i,
             adc_pattern[i].channel);
  }
  config.adc_pattern = adc_pattern;
  ESP_ERROR_CHECK(adc_continuous_config(adc_handle, &config));

  adc_continuous_evt_cbs_t callbacks = {
    .on_conv_done = on_conversion_done_cb,
  };
  ESP_ERROR_CHECK(
      adc_continuous_register_event_callbacks(adc_handle, &callbacks, NULL));
}

void adc_task(void *pvParameters) {
  adc_task_handle = xTaskGetCurrentTaskHandle();
  uint32_t conversion_frame_real_size = 0;
  uint8_t conversions[CONVERSION_FRAME_SIZE] = { 0 };
  memset(conversions, 0, CONVERSION_FRAME_SIZE);

  while (1) {
    ESP_ERROR_CHECK(adc_continuous_start(adc_handle));
    vTaskDelay(1);
    ulTaskNotifyTake(pdTRUE, portMAX_DELAY);

    adc_continuous_read(adc_handle, conversions, CONVERSION_FRAME_SIZE,
                        &conversion_frame_real_size, 0);

    ESP_ERROR_CHECK(adc_continuous_stop(adc_handle));

    for (uint8_t conversion_result_index = 0;
         conversion_result_index < conversion_frame_real_size;
         conversion_result_index += SOC_ADC_DIGI_DATA_BYTES_PER_CONV) {
      adc_digi_output_data_t *conversion_frame =
          (adc_digi_output_data_t *)&conversions[conversion_result_index];
      for (uint8_t adc_channel = 0; adc_channel < ADC_CHANNEL_COUNT;
           adc_channel++) {
        if (conversion_frame->type2.channel == adc_channels[adc_channel]) {
          if (adc_channels[adc_channel] == ADC_CHANNEL_0) {
            update_battery_voltage(conversion_frame->type2.data);
          } else {
            update_key_state(adc_channel, conversion_frame->type2.data);
          }
          break;
        }
      }
    }
  }
}
