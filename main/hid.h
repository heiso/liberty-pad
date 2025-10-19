#pragma once

#include "esp_err.h"
#include <stdbool.h>
#include <stdint.h>

// Common HID key codes
#define HID_KEY_UP 0x52
#define HID_KEY_DOWN 0x51
#define HID_KEY_LEFT 0x50
#define HID_KEY_RIGHT 0x4F

/**
 * @brief Initialize BLE HID keyboard
 *
 * This function initializes the Bluetooth controller, Bluedroid stack,
 * HID profile, and sets up advertising for the HID keyboard device.
 *
 * @return esp_err_t ESP_OK on success, error code otherwise
 */
esp_err_t hid_init(void);

/**
 * @brief Check if HID device is connected and paired
 *
 * @return true if connected and secure connection established
 * @return false if not connected or not paired
 */
bool hid_is_connected(void);

/**
 * @brief Send a key press or release
 *
 * @param keycode HID keycode (see HID_KEY_* defines)
 * @return esp_err_t ESP_OK on success, error code otherwise
 */
esp_err_t hid_send_key(uint8_t keycode);

/**
 * @brief Send an empty key report to release all keys
 *
 * @param keycode HID keycode (see HID_KEY_* defines)
 * @param pressed true to press the key, false to release
 * @return esp_err_t ESP_OK on success, error code otherwise
 */
esp_err_t hid_send_empty();

esp_err_t hid_send_keycodes(uint8_t keycode, size_t length);