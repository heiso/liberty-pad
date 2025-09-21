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

struct key keys[ADC_CHANNEL_COUNT] = { 0 };

void init_keys() {
  for (int i = 0; i < ADC_CHANNEL_COUNT; i++) {
    keys[i].config.hardware.adc_channel = adc_channels[i];
    keys[i].config.hardware.magnet_polarity = NORTH_POLE_FACING_DOWN;

    keys[i].config.deadzones.start_offset = 17;
    keys[i].config.deadzones.end_offset = 17;
    keys[i].config.actuation_distance = 63; // 1mm
    keys[i].config.release_distance = 50;   // <1mm

    keys[i].config.rapid_trigger.is_enabled = 1;
    keys[i].config.rapid_trigger.is_continuous = 1;
    keys[i].config.rapid_trigger.actuation_distance_delta = 31;
    keys[i].config.rapid_trigger.release_distance_delta = 31;

    keys[i].calibration.max_distance = MAX_DISTANCE_PRE_CALIBRATION;
    keys[i].status = STATUS_RESET;
  }
  keys[0].config.hardware.adc_channel = ADC_CHANNEL_3; // up
  keys[0].config.keycode = 0x52;                       // Up Arrow
  keys[1].config.hardware.adc_channel = ADC_CHANNEL_4; // left
  keys[1].config.keycode = 0x50;                       // Left Arrow
  keys[2].config.hardware.adc_channel = ADC_CHANNEL_5; // down
  keys[2].config.keycode = 0x51;                       // Down Arrow
  keys[3].config.hardware.adc_channel = ADC_CHANNEL_6; // right
  keys[3].config.keycode = 0x4F;                       // Right Arrow
  keys[3].config.hardware.magnet_polarity = SOUTH_POLE_FACING_DOWN;
}

void update_key_state(adc_channel_t adc_channel, uint16_t raw_value) {
  struct key_state new_state = { 0 };

  uint16_t normalized_value = 0;
  if (keys[adc_channel].config.hardware.magnet_polarity == NORTH_POLE_FACING_DOWN) {
    normalized_value = ADC_VREF - raw_value;
  } else {
    normalized_value = raw_value;
  }

  // new_state.raw_adc_value = normalized_value;

  // Initial calibration of IDLE value
  // Only for the first 2 seconds after task start
  if (xTaskGetTickCount() < pdMS_TO_TICKS(1000)) {
    if (keys[adc_channel].calibration.idle_value == 0) {
      keys[adc_channel].calibration.idle_value = normalized_value;
    } else {
      float delta = 0.6;
      keys[adc_channel].calibration.idle_value = (1 - delta) * normalized_value + delta * keys[adc_channel].calibration.idle_value;
    }

    keys[adc_channel].state = new_state;
    return;
  }

  // Calibrate idle value
  if (normalized_value < keys[adc_channel].calibration.idle_value) {
    float delta = 0.8;
    keys[adc_channel].calibration.idle_value = (1 - delta) * normalized_value + delta * keys[adc_channel].calibration.idle_value;
  }

  uint16_t distance = 0;
  // Get distance
  if (normalized_value > keys[adc_channel].calibration.idle_value) {
    distance = normalized_value - keys[adc_channel].calibration.idle_value;
  } else {
    distance = 0;
  }

  // Calibrate max distance value
  if (distance > keys[adc_channel].calibration.max_distance) {
    keys[adc_channel].calibration.max_distance = distance;
  }

  // Get 8-bit distance
  if (distance >= keys[adc_channel].calibration.max_distance - keys[adc_channel].config.deadzones.end_offset) {
    new_state.distance = 255;
    keys[adc_channel].is_idle = 0;
  } else if (distance <= keys[adc_channel].config.deadzones.start_offset) {
    new_state.distance = 0;
  } else {
    new_state.distance = (distance * 255) / keys[adc_channel].calibration.max_distance;
    keys[adc_channel].is_idle = 0;
  }

  // // Update velocity
  // new_state.velocity = new_state.distance - keys[adc_channel].state.distance;

  // new_state.acceleration = new_state.velocity - keys[adc_channel].state.velocity;
  // new_state.jerk = new_state.acceleration - keys[adc_channel].state.acceleration;

  // // This should be moved in another function dedicated to update the direction and trigger/reset state. Maybe some state machine?
  // // Not needed for this project as everykey is mutually exclusive (kind of SOCD)
  // enum key_direction previous_direction = keys[adc_channel].direction;
  // // Update direction
  // if (new_state.distance == 0) {
  //   keys[adc_channel].direction = UP;
  //   // keys[adc_channel].from = 0;
  // }
  // if (keys[adc_channel].since == 0 || xTaskGetTickCount() - keys[adc_channel].since > pdMS_TO_TICKS(MIN_TIME_BETWEEN_DIRECTION_CHANGE_MS)) {
  //   if (new_state.velocity > 0 && keys[adc_channel].state.velocity > 0 && keys[adc_channel].direction != DOWN) {
  //     keys[adc_channel].direction = DOWN;
  //     // if (keys[adc_channel].state.from != 0) {
  //     //   keys[adc_channel].from = keys[adc_channel].state.distance;
  //     // }
  //   } else if (new_state.velocity < 0 && keys[adc_channel].state.velocity > 0 && keys[adc_channel].direction != UP) {
  //     keys[adc_channel].direction = UP;
  //     // if (keys[adc_channel].state.from != 255) {
  //     //   keys[adc_channel].from = keys[adc_channel].state.distance;
  //     // }
  //   }
  // }
  // if (keys[adc_channel].direction != previous_direction || new_state.distance == 0) {
  //   keys[adc_channel].since = xTaskGetTickCount();
  // }

  keys[adc_channel].state = new_state;
}

void update_keys() {
  for (int i = 0; i < ADC_CHANNEL_COUNT; i++) {
    switch (keys[i].status) {
    case STATUS_RESET:
      if (keys[i].state.distance >= keys[i].config.actuation_distance) {
        keys[i].status = STATUS_TRIGGERED;
        keys[i].triggered_at = xTaskGetTickCount();
      }
      break;
    case STATUS_TRIGGERED:
      if (keys[i].state.distance <= keys[i].config.release_distance) {
        keys[i].status = STATUS_RESET;
        keys[i].triggered_at = 0;
      }
      break;
    default:
      break;
    }
  }
}

uint8_t get_last_triggered_key_index() {
  uint8_t last_triggered_key = 255;
  uint32_t last_triggered_at = 0;
  for (int i = 0; i < ADC_CHANNEL_COUNT; i++) {
    if (keys[i].status == STATUS_TRIGGERED) {
      if (keys[i].triggered_at > last_triggered_at) {
        last_triggered_at = keys[i].triggered_at;
        last_triggered_key = i;
      }
    }
  }
  return last_triggered_key;
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
    update_keys();

    printf("\033[H\033[J");
    // for (uint8_t adc_channel = 0; adc_channel < ADC_CHANNEL_COUNT;
    //      adc_channel++) {
    // printf("%1d | ", adc_channel);
    // // printf("raw: %4d, ", keys[adc_channel].raw_adc_value);
    // printf("cal_idle: %4d, ", keys[adc_channel].calibration.idle_value);
    // printf("cal_max: %4d, ", keys[adc_channel].calibration.max_distance);
    // // printf("dist: %4d, ", keys[adc_channel].state.distance);
    // printf("dist8: %3d, ", keys[adc_channel].state.distance);
    // printf("velo: %3d, ", keys[adc_channel].state.velocity);
    // printf("acc: %3d, ", keys[adc_channel].state.acceleration);
    // printf("jerk: %3d, ", keys[adc_channel].state.jerk);
    // printf("dir: ");
    // if (keys[adc_channel].direction == DOWN) {
    //   printf("DOWN, ");
    // } else if (keys[adc_channel].direction == UP) {
    //   printf("UP, ");
    // } else {
    //   printf("%1d, ", keys[adc_channel].direction);
    // }
    // printf("start_offset: %2d, ", keys[adc_channel].config.deadzones.start_offset);
    // printf("end_offset: %2d, ", keys[adc_channel].config.deadzones.end_offset);
    // }
    uint8_t index = get_last_triggered_key_index();
    if (index != 255) {
      printf("%3d", index);
    }
    printf("\n");

    vTaskDelay(pdMS_TO_TICKS(50));
  }
}

void app_main(void) {
  adc_init();
  init_keys();

  xTaskCreate(adc_task, "adc_task", 4096, NULL, 10, NULL);
  xTaskCreate(debug_task, "debug_task", 2048, NULL, 5, NULL);
}
