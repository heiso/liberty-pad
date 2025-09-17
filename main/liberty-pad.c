#include "liberty-pad.h"
#include "esp_adc/adc_continuous.h"
#include "esp_log.h"
#include "freertos/FreeRTOS.h"
#include "freertos/queue.h"
#include "freertos/task.h"
#include "sdkconfig.h"
#include <stdio.h>
#include <string.h>

static const char *TAG = "LIBERTY_PAD";

#define ADC_CHANNEL_COUNT 4
#define CONVERSION_FRAME_SIZE (SOC_ADC_DIGI_DATA_BYTES_PER_CONV * ADC_CHANNEL_COUNT)
#define CONVERSION_POOL_SIZE CONVERSION_FRAME_SIZE * 1
#define ADC_VREF 3300
#define MAX_DISTANCE_PRE_CALIBRATION 500
#define MIN_TIME_BETWEEN_DIRECTION_CHANGE_MS 100

const uint32_t adc_channels[ADC_CHANNEL_COUNT] = {
  ADC_CHANNEL_3, // up
  ADC_CHANNEL_4, // left
  ADC_CHANNEL_5, // down
  ADC_CHANNEL_6, // right
};

adc_continuous_handle_t adc_handle;
static TaskHandle_t adc_task_handle;

struct key_config key_configs[ADC_CHANNEL_COUNT] = { 0 };
struct key_state key_states[ADC_CHANNEL_COUNT] = { 0 };

void init_keys() {
  for (int i = 0; i < ADC_CHANNEL_COUNT; i++) {
    key_configs[i].hardware.adc_channel = adc_channels[i];
    key_configs[i].hardware.magnet_polarity = NORTH_POLE_FACING_DOWN;

    key_configs[i].deadzones.start_offset = 17;
    key_configs[i].deadzones.end_offset = 17;
    key_configs[i].actuation_distance = 63; // 1mm
    key_configs[i].release_distance = 50;   // <1mm

    key_configs[i].rapid_trigger.is_enabled = 1;
    key_configs[i].rapid_trigger.is_continuous = 1;
    key_configs[i].rapid_trigger.actuation_distance_delta = 31;
    key_configs[i].rapid_trigger.release_distance_delta = 31;
  }
  key_configs[0].hardware.adc_channel = ADC_CHANNEL_3; // up
  key_configs[0].keycode = 0x52;                       // Up Arrow
  key_configs[1].hardware.adc_channel = ADC_CHANNEL_4; // left
  key_configs[1].keycode = 0x50;                       // Left Arrow
  key_configs[2].hardware.adc_channel = ADC_CHANNEL_5; // down
  key_configs[2].keycode = 0x51;                       // Down Arrow
  key_configs[3].hardware.adc_channel = ADC_CHANNEL_6; // right
  key_configs[3].keycode = 0x4F;                       // Right Arrow
  key_configs[3].hardware.magnet_polarity = SOUTH_POLE_FACING_DOWN;
}

void update_key_state(adc_channel_t adc_channel, uint16_t raw_value) {
  struct key_state new_state = { 0 };
  // Maybe this calibration data should be stored somewhere else?
  new_state.calibration.idle_value = key_states[adc_channel].calibration.idle_value;
  new_state.calibration.max_distance = key_states[adc_channel].calibration.max_distance;
  if (new_state.calibration.max_distance == 0) {
    new_state.calibration.max_distance = MAX_DISTANCE_PRE_CALIBRATION;
  }

  // This should also be copied from previous state, maybe there is a better way to do this?
  new_state.is_idle = key_states[adc_channel].is_idle;
  new_state.direction = key_states[adc_channel].direction;
  new_state.from = key_states[adc_channel].from;
  new_state.since = key_states[adc_channel].since;

  if (key_configs[adc_channel].hardware.magnet_polarity == NORTH_POLE_FACING_DOWN) {
    new_state.raw_adc_value = ADC_VREF - raw_value;
  } else {
    new_state.raw_adc_value = raw_value;
  }

  // Initial calibration of IDLE value
  // Only for the first 2 seconds after task start
  if (xTaskGetTickCount() < pdMS_TO_TICKS(1000)) {
    if (key_states[adc_channel].calibration.idle_value == 0) {
      new_state.calibration.idle_value = new_state.raw_adc_value;
    } else {
      float delta = 0.6;
      new_state.calibration.idle_value = (1 - delta) * new_state.raw_adc_value + delta * key_states[adc_channel].calibration.idle_value;
    }

    key_states[adc_channel] = new_state;
    return;
  }

  // Calibrate idle value
  if (new_state.raw_adc_value < key_states[adc_channel].calibration.idle_value) {
    float delta = 0.8;
    new_state.calibration.idle_value = (1 - delta) * new_state.raw_adc_value + delta * key_states[adc_channel].calibration.idle_value;
    new_state.raw_adc_value = key_states[adc_channel].calibration.idle_value;
  }

  // Get distance
  if (new_state.raw_adc_value > new_state.calibration.idle_value) {
    new_state.distance = new_state.raw_adc_value - new_state.calibration.idle_value;
  } else {
    new_state.distance = 0;
  }

  // Calibrate max distance value
  if (new_state.distance > new_state.calibration.max_distance) {
    new_state.calibration.max_distance = new_state.distance;
  }

  // Get 8-bit distance
  if (new_state.distance >= new_state.calibration.max_distance - key_configs[adc_channel].deadzones.end_offset) {
    new_state.distance_8bits = 255;
    new_state.is_idle = 0;
  } else if (new_state.distance <= key_configs[adc_channel].deadzones.start_offset) {
    new_state.distance_8bits = 0;
  } else {
    new_state.distance_8bits = (new_state.distance * 255) / new_state.calibration.max_distance;
    new_state.is_idle = 0;
  }

  // Update velocity
  new_state.velocity = new_state.distance_8bits - key_states[adc_channel].distance_8bits;

  new_state.acceleration = new_state.velocity - key_states[adc_channel].velocity;
  new_state.jerk = new_state.acceleration - key_states[adc_channel].acceleration;

  // Update direction
  if (new_state.distance_8bits == 0) {
    new_state.direction = UP;
    // new_state.from = 0;
  }

  if (key_states[adc_channel].since == 0 || xTaskGetTickCount() - key_states[adc_channel].since > pdMS_TO_TICKS(MIN_TIME_BETWEEN_DIRECTION_CHANGE_MS)) {
    if (new_state.velocity > 0 && key_states[adc_channel].velocity > 0 && key_states[adc_channel].direction != DOWN) {
      new_state.direction = DOWN;
      // if (key_states[adc_channel].from != 0) {
      //   new_state.from = key_states[adc_channel].distance_8bits;
      // }
    } else if (new_state.velocity < 0 && key_states[adc_channel].velocity > 0 && key_states[adc_channel].direction != UP) {
      new_state.direction = UP;
      // if (key_states[adc_channel].from != 255) {
      //   new_state.from = key_states[adc_channel].distance_8bits;
      // }
    }
  }

  if (new_state.direction != key_states[adc_channel].direction || new_state.distance == 0) {
    new_state.since = xTaskGetTickCount();
  }

  key_states[adc_channel] = new_state;
}

static bool IRAM_ATTR
on_conversion_done_cb(adc_continuous_handle_t handle,
                      const adc_continuous_evt_data_t *edata, void *user_data) {
  BaseType_t mustYield = pdFALSE;
  // Notify that ADC continuous driver has done enough number of conversions
  vTaskNotifyGiveFromISR(adc_task_handle, &mustYield);

  return (mustYield == pdTRUE);
}

static void adc_init() {
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
          update_key_state(adc_channel, conversion_frame->type2.data);
          break;
        }
      }
    }
  }
}

void debug_task(void *pvParameters) {
  while (1) {

    // printf("\033[H\033[J");
    // for (uint8_t adc_channel = 0; adc_channel < ADC_CHANNEL_COUNT;
    //      adc_channel++) {
    //   printf("%1d | ", adc_channel);
    //   printf("raw: %4d, ", key_states[adc_channel].raw_adc_value);
    //   printf("cal_idle: %4d, ", key_states[adc_channel].calibration.idle_value);
    //   printf("cal_max: %4d, ", key_states[adc_channel].calibration.max_distance);
    //   printf("dist: %4d, ", key_states[adc_channel].distance);
    //   printf("dist8: %3d, ", key_states[adc_channel].distance_8bits);
    //   printf("dist: %3d, ", key_states[adc_channel].distance_8bits);
    //   printf("velo: %3d, ", key_states[adc_channel].velocity);
    //   printf("acc: %3d, ", key_states[adc_channel].acceleration);
    //   printf("jerk: %3d, ", key_states[adc_channel].jerk);
    //   printf("dir: ");
    //   if (key_states[adc_channel].direction == DOWN) {
    //     printf("DOWN, ");
    //   } else if (key_states[adc_channel].direction == UP) {
    //     printf("UP, ");
    //   } else {
    //     printf("%1d, ", key_states[adc_channel].direction);
    //   }
    //   printf("start_offset: %2d, ", key_configs[adc_channel].deadzones.start_offset);
    //   printf("end_offset: %2d, ", key_configs[adc_channel].deadzones.end_offset);
    //   printf("\n");
    // }
    printf("dist:");
    printf("%d", key_states[0].distance_8bits);
    printf(",");
    printf("velo:");
    printf("%d", key_states[0].velocity);
    printf(",");
    printf("dir:");
    printf("%d", key_states[0].direction);
    // printf("acc:");
    // printf("%d", key_states[0].acceleration);
    // printf(",");
    // printf("jerk:");
    // printf("%d", key_states[0].jerk);
    printf("\n");

    vTaskDelay(pdMS_TO_TICKS(5));
  }
}

void app_main(void) {
  adc_init();
  init_keys();

  xTaskCreate(adc_task, "adc_task", 4096, NULL, 10, NULL);
  xTaskCreate(debug_task, "debug_task", 2048, NULL, 5, NULL);
}
