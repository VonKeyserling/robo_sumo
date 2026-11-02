#pragma once
#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>
#include "esp_err.h"
#include "cyclone2_parser.h"

#ifdef __cplusplus
extern "C" {
#endif

typedef enum {
    CYCLONE2_CONNECTED,
    CYCLONE2_DISCONNECTED,
    CYCLONE2_INPUT,
} cyclone2_event_type_t;

typedef struct {
    cyclone2_event_type_t type;
    // Relevant for CYCLONE2_INPUT only. `data` is valid only during callback.
    uint16_t report_id;
    uint8_t map_index;
    const uint8_t *data;
    size_t length;
    cyclone2_gamepad_t gamepad;  // decoded=false when unknown layout.
} cyclone2_event_t;

typedef void (*cyclone2_callback_t)(const cyclone2_event_t *event, void *arg);

/* Initializes NVS-independent Bluetooth Classic + Bluedroid + HID Host and
 * retries a connection to the supplied BD_ADDR every ~5 s after failures.
 * MAC example: "AA:BB:CC:DD:EE:FF". Call once from app_main().
 * Callback runs in an ESP-IDF event task: keep it short and nonblocking.
 */
esp_err_t cyclone2_start(const char *mac, cyclone2_callback_t cb, void *arg);
bool cyclone2_is_connected(void);
/* Thread-safe snapshot: false (and zeroed state) if input is stale >500ms. */
bool cyclone2_get_gamepad(cyclone2_gamepad_t *out);

#ifdef __cplusplus
}
#endif
