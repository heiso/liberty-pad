#pragma once

#include "esp_err.h"
#include <stdbool.h>
#include <stdint.h>

// HID Key codes
#define HID_KEY_A 4
#define HID_KEY_B 5
#define HID_KEY_C 6
#define HID_KEY_D 7
#define HID_KEY_E 8
#define HID_KEY_F 9
#define HID_KEY_G 10
#define HID_KEY_H 11
#define HID_KEY_I 12
#define HID_KEY_J 13
#define HID_KEY_K 14
#define HID_KEY_L 15
#define HID_KEY_M 16
#define HID_KEY_N 17
#define HID_KEY_O 18
#define HID_KEY_P 19
#define HID_KEY_Q 20
#define HID_KEY_R 21
#define HID_KEY_S 22
#define HID_KEY_T 23
#define HID_KEY_U 24
#define HID_KEY_V 25
#define HID_KEY_W 26
#define HID_KEY_X 27
#define HID_KEY_Y 28
#define HID_KEY_Z 29
#define HID_KEY_1 30
#define HID_KEY_2 31
#define HID_KEY_3 32
#define HID_KEY_4 33
#define HID_KEY_5 34
#define HID_KEY_6 35
#define HID_KEY_7 36
#define HID_KEY_8 37
#define HID_KEY_9 38
#define HID_KEY_0 39
#define HID_KEY_RETURN 40
#define HID_KEY_ESCAPE 41
#define HID_KEY_DELETE 42
#define HID_KEY_TAB 43
#define HID_KEY_SPACEBAR 44
#define HID_KEY_UP 82
#define HID_KEY_DOWN 81
#define HID_KEY_LEFT 80
#define HID_KEY_RIGHT 79

// Consumer control usage codes
#define HID_CONSUMER_PLAY_PAUSE 205
#define HID_CONSUMER_VOLUME_UP 233
#define HID_CONSUMER_VOLUME_DOWN 234
#define HID_CONSUMER_MUTE 226
#define HID_CONSUMER_SCAN_NEXT_TRK 181
#define HID_CONSUMER_SCAN_PREV_TRK 182

// Modifier key masks
#define HID_KEY_LEFT_CTRL (1 << 0)
#define HID_KEY_LEFT_SHIFT (1 << 1)
#define HID_KEY_LEFT_ALT (1 << 2)
#define HID_KEY_LEFT_GUI (1 << 3)
#define HID_KEY_RIGHT_CTRL (1 << 4)
#define HID_KEY_RIGHT_SHIFT (1 << 5)
#define HID_KEY_RIGHT_ALT (1 << 6)
#define HID_KEY_RIGHT_GUI (1 << 7)

/**
 * @brief Initialize BLE HID device
 */
esp_err_t hid_init(void);

/**
 * @brief Check if HID device is connected and paired
 */
bool hid_is_connected(void);

/**
 * @brief Get the current HID connection ID
 */
uint16_t hid_get_conn_id(void);

/**
 * @brief Send multiple keyboard keys with modifiers
 * @param modifier Modifier key mask
 * @param keycodes Array of key codes (up to 6 keys)
 * @param keycodes_length Number of keys in array (max 6)
 */
esp_err_t hid_send_keys(uint8_t modifier, uint8_t keycodes[6], uint8_t keycodes_length);

/**
 * @brief Send consumer control command (media keys)
 * @param usage_code Consumer usage code (HID_CONSUMER_*)
 */
esp_err_t hid_send_consumer(uint16_t usage_code);