#include "main.h"
#include "esp_log.h"
#include "freertos/FreeRTOS.h"
#include "freertos/queue.h"
#include "freertos/task.h"
#include "hid.h"
#include "sdkconfig.h"
#include "sensor.h"
#include <stdio.h>
#include <string.h>

static const char *TAG = "LIBERTY_PAD";

#define KEYS_COUNT 4
#define ADC_VREF 3300
#define MAX_DISTANCE_PRE_CALIBRATION 500
#define MIN_TIME_BETWEEN_DIRECTION_CHANGE_MS 100

const uint32_t adc_channels[ADC_CHANNEL_COUNT] = {
  ADC_CHANNEL_3,
  ADC_CHANNEL_4,
  ADC_CHANNEL_5,
  ADC_CHANNEL_6,
  ADC_CHANNEL_0, // Battery voltage on ADC_CHANNEL_0
};

struct key keys[KEYS_COUNT] = { 0 };

void init_keys() {
  for (int i = 0; i < KEYS_COUNT; i++) {
    keys[i].config.hardware.adc_channel = adc_channels[i];
    keys[i].config.hardware.magnet_polarity = NORTH_POLE_FACING_DOWN;

    keys[i].config.deadzones.start_offset = 17;
    keys[i].config.deadzones.end_offset = 17;
    keys[i].config.actuation_distance = 128;
    keys[i].config.release_distance = 127;

    keys[i].config.rapid_trigger.is_enabled = 1;
    keys[i].config.rapid_trigger.is_continuous = 1;
    keys[i].config.rapid_trigger.actuation_distance_delta = 31;
    keys[i].config.rapid_trigger.release_distance_delta = 31;

    keys[i].calibration.max_distance = MAX_DISTANCE_PRE_CALIBRATION;
    keys[i].status = STATUS_RESET;
  }
  keys[0].config.hardware.adc_channel = ADC_CHANNEL_3;
  keys[0].config.keycode = HID_KEY_RIGHT;
  keys[1].config.hardware.adc_channel = ADC_CHANNEL_4;
  keys[1].config.keycode = HID_KEY_LEFT;
  keys[2].config.hardware.adc_channel = ADC_CHANNEL_5;
  keys[2].config.keycode = HID_KEY_DOWN;
  keys[3].config.hardware.adc_channel = ADC_CHANNEL_6;
  keys[3].config.keycode = HID_KEY_UP;
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

  keys[adc_channel].state = new_state;
}

void update_key_direction(struct key *key) {
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
}

void update_keys(void *pvParameters) {
  while (1) {
    static uint8_t should_send_report = 0;

    uint8_t keycodes[6] = { 0 };
    uint8_t keycodes_length = 0;

    for (int i = 0; i < ADC_CHANNEL_COUNT; i++) {
      update_key_direction(&keys[i]);

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

      if (keys[i].status == STATUS_TRIGGERED) {
        keycodes[keycodes_length] = keys[i].config.keycode;
        keycodes_length++;
      }
    }

    if (should_send_report) {
      hid_send_keycodes(*keycodes, keycodes_length);
    }

    if (keycodes_length > 0) {
      should_send_report = 1;
    } else {
      should_send_report = 0;
    }

    vTaskDelay(pdMS_TO_TICKS(10));
  }
}

void debug_task(void *pvParameters) {
  while (1) {
    // printf("\033[H\033[J");
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

    vTaskDelay(pdMS_TO_TICKS(50));
  }
}

void update_battery_voltage(uint16_t raw_value) {
  uint8_t battery_lev = 0;
  uint16_t voltage = raw_value * 2;
  // Convert voltage to percentage (assuming 3.0V min, 4.2V max for Li-ion)
  if (voltage >= 4200) {
    battery_lev = 100;
  } else if (voltage <= 3000) {
    battery_lev = 0;
  } else {
    battery_lev = ((voltage - 3000) * 100) / (4200 - 3000);
  }
  ESP_LOGI(TAG, "Battery voltage: %dmV", voltage);
  ESP_LOGI(TAG, "Battery level: %d%%", battery_lev);
  // Here you can add code to handle low battery warnings or other logic
}

void app_main(void) {
  // Initialize HID first
  esp_err_t ret = hid_init();
  if (ret != ESP_OK) {
    ESP_LOGE(TAG, "Failed to initialize HID: %s", esp_err_to_name(ret));
    return;
  }

  adc_init();
  init_keys();

  xTaskCreate(adc_task, "adc_task", 4096, NULL, 10, NULL);
  xTaskCreate(update_keys, "update_keys", 2048, NULL, 10, NULL);
  // xTaskCreate(debug_task, "debug_task", 2048, NULL, 5, NULL);
}
