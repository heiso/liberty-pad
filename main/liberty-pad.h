#pragma once

#include <stdint.h>

struct switch_magnetic_profile {
  uint8_t id;
  uint16_t adc_reading_by_distance[255];
};

enum magnet_polarity {
  NORTH_POLE_FACING_DOWN,
  SOUTH_POLE_FACING_DOWN,
};

struct deadzones {
  uint8_t start_offset;
  uint8_t end_offset;
};

struct rapid_trigger {
  uint8_t is_enabled;
  uint8_t is_continuous;
  uint8_t actuation_distance_delta;
  uint8_t release_distance_delta;
};

struct hardware {
  uint8_t adc_channel;
  enum magnet_polarity magnet_polarity;
};

struct key_config {
  struct hardware hardware;

  struct deadzones deadzones;
  uint8_t actuation_distance;
  uint8_t release_distance;
  struct rapid_trigger rapid_trigger;

  uint16_t keycode;
};

struct calibration {
  uint16_t idle_value;
  uint16_t max_distance;
};

enum direction {
  UP,
  DOWN,
};

struct key_state {
  uint8_t is_idle;

  struct calibration calibration;

  uint16_t raw_adc_value;
  uint16_t distance;
  uint8_t distance_8bits;
  int8_t velocity;
  int8_t acceleration;
  int8_t jerk;
  // Distance from where the travel has begun
  uint8_t from;
  // Time since the travel has begun
  uint32_t since;
  enum direction direction;
};

// Switch profile lookup table
// extern const uint8_t switch_profile[3301];