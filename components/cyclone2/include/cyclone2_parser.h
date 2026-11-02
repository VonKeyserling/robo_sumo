#pragma once
#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

// Application button bits (not positions within the HID packet).
enum {
  CYCLONE2_BTN_A = (1u << 0),
  CYCLONE2_BTN_B = (1u << 1),
  CYCLONE2_BTN_X = (1u << 2),
  CYCLONE2_BTN_Y = (1u << 3),
  CYCLONE2_BTN_LB = (1u << 4),
  CYCLONE2_BTN_RB = (1u << 5),
  CYCLONE2_BTN_LT = (1u << 6),
  CYCLONE2_BTN_RT = (1u << 7),
  CYCLONE2_BTN_VIEW = (1u << 8),
  CYCLONE2_BTN_MENU = (1u << 9),
  CYCLONE2_BTN_L3 = (1u << 10),
  CYCLONE2_BTN_R3 = (1u << 11),
  CYCLONE2_BTN_HOME = (1u << 12),
  CYCLONE2_BTN_SHARE = (1u << 13),
  CYCLONE2_BTN_L4 = (1u << 14),
  CYCLONE2_BTN_R4 = (1u << 15),
  CYCLONE2_BTN_M = (1u << 16),
};

typedef enum {
  CYCLONE2_LAYOUT_UNKNOWN = 0,
  CYCLONE2_LAYOUT_ANDROID_DINPUT_01, // Standard 9-byte candidate for this
                                     // personality
  CYCLONE2_LAYOUT_EXTENDED_12,       // Enhanced/GameSir DS4-style format
  CYCLONE2_LAYOUT_ANDROID_HID_07,    // Bluetooth report 0x07, 10 payload bytes
} cyclone2_layout_t;

typedef struct {
  bool decoded;               // false = do not use this as a gamepad state
  bool physical_map_verified; // false for Android 0x01 until confirmed with
                              // this firmware
  cyclone2_layout_t layout;
  uint16_t report_id;
  uint8_t raw_lx, raw_ly, raw_rx, raw_ry; // 0..255, 128 near center
  int16_t lx, ly, rx, ry;                 // ~-32767 .. +32767; positive Y = UP
  uint8_t lt, rt;                         // 0..255 (analogue)
  uint8_t hat;                // 0..7 clockwise from N, 8/15 = neutral
  bool up, down, left, right; // D-pad directions
  uint16_t raw_buttons;       // raw bitmap on Android report 0x01
  uint32_t buttons;           // normalized CYCLONE2_BTN_* bits
  bool a, b, x, y;
  bool lb, rb, view, menu, l3, r3, home, share;
  bool l4, r4, m;
} cyclone2_gamepad_t;

// data excludes the report ID: ESP-IDF esp_hidh.input.report_id is separate.
// Unrecognized formats return false and reset `out` (decoded=false).
bool cyclone2_parse(uint16_t report_id, const uint8_t *data, size_t len,
                    cyclone2_gamepad_t *out);

// Backwards-compatible entry point; processes only report 0x01.
bool cyclone2_parse_dinput01(uint16_t report_id, const uint8_t *data,
                             size_t len, cyclone2_gamepad_t *out);

static inline bool cyclone2_button(const cyclone2_gamepad_t *g, uint32_t mask) {
  return g != NULL && g->decoded && (g->buttons & mask) != 0;
}

#ifdef __cplusplus
}
#endif
