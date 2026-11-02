#include "bts7960.h"

#include <stdbool.h>

#include "driver/gpio.h"
#include "driver/mcpwm_prelude.h"
#include "esp_log.h"

#define PIN_RPWM GPIO_NUM_25
#define PIN_LPWM GPIO_NUM_26
#define PIN_REN  GPIO_NUM_27
#define PIN_LEN  GPIO_NUM_14

// Timer resolution 10MHz; PWM at 20kHz: 10,000,000 / 20,000 = 500 ticks.
#define PWM_RESOLUTION_HZ 10000000
#define PWM_PERIOD_TICKS 500
#define MCPWM_GROUP 1 // Original ESP32 has groups 0 and 1.

static const char *TAG = "BTS7960";
static mcpwm_timer_handle_t s_timer;
static mcpwm_oper_handle_t s_oper;
static mcpwm_cmpr_handle_t s_cmp_r;
static mcpwm_cmpr_handle_t s_cmp_l;
static mcpwm_gen_handle_t s_gen_r;
static mcpwm_gen_handle_t s_gen_l;
static bool s_ready = false;
static int s_current_percent = 0;

// Internal: return errors rather than crashing at a PWM reconfiguration step.
#define RETURN_IF_ERR(expr) do { esp_err_t e_ = (expr); if (e_ != ESP_OK) return e_; } while (0)

static void force_both_low(void)
{
    ESP_ERROR_CHECK(mcpwm_generator_set_force_level(s_gen_r, 0, true));
    ESP_ERROR_CHECK(mcpwm_generator_set_force_level(s_gen_l, 0, true));
}

static esp_err_t configure_wave(mcpwm_gen_handle_t gen, mcpwm_cmpr_handle_t cmp)
{
    RETURN_IF_ERR(mcpwm_generator_set_action_on_timer_event(
        gen, MCPWM_GEN_TIMER_EVENT_ACTION(
            MCPWM_TIMER_DIRECTION_UP, MCPWM_TIMER_EVENT_EMPTY, MCPWM_GEN_ACTION_HIGH)));
    RETURN_IF_ERR(mcpwm_generator_set_action_on_compare_event(
        gen, MCPWM_GEN_COMPARE_EVENT_ACTION(
            MCPWM_TIMER_DIRECTION_UP, cmp, MCPWM_GEN_ACTION_LOW)));
    return ESP_OK;
}

esp_err_t bts7960_init(void)
{
    if (s_ready) return ESP_ERR_INVALID_STATE;

    // Keep EN low throughout initialization, before PWM pins are configured.
    gpio_config_t io = {
        .pin_bit_mask = (1ULL << PIN_REN) | (1ULL << PIN_LEN),
        .mode = GPIO_MODE_OUTPUT,
        .pull_down_en = GPIO_PULLDOWN_DISABLE,
        .pull_up_en = GPIO_PULLUP_DISABLE,
        .intr_type = GPIO_INTR_DISABLE,
    };
    RETURN_IF_ERR(gpio_config(&io));
    RETURN_IF_ERR(gpio_set_level(PIN_REN, 0));
    RETURN_IF_ERR(gpio_set_level(PIN_LEN, 0));

    mcpwm_timer_config_t timer_cfg = {
        .group_id = MCPWM_GROUP,
        .clk_src = MCPWM_TIMER_CLK_SRC_DEFAULT,
        .resolution_hz = PWM_RESOLUTION_HZ,
        .period_ticks = PWM_PERIOD_TICKS,
        .count_mode = MCPWM_TIMER_COUNT_MODE_UP,
    };
    RETURN_IF_ERR(mcpwm_new_timer(&timer_cfg, &s_timer));

    mcpwm_operator_config_t oper_cfg = { .group_id = MCPWM_GROUP };
    RETURN_IF_ERR(mcpwm_new_operator(&oper_cfg, &s_oper));
    RETURN_IF_ERR(mcpwm_operator_connect_timer(s_oper, s_timer));

    mcpwm_comparator_config_t cmp_cfg = {
        .flags.update_cmp_on_tez = true,
    };
    RETURN_IF_ERR(mcpwm_new_comparator(s_oper, &cmp_cfg, &s_cmp_r));
    RETURN_IF_ERR(mcpwm_new_comparator(s_oper, &cmp_cfg, &s_cmp_l));

    mcpwm_generator_config_t gen_r_cfg = {.gen_gpio_num = PIN_RPWM};
    mcpwm_generator_config_t gen_l_cfg = {.gen_gpio_num = PIN_LPWM};
    RETURN_IF_ERR(mcpwm_new_generator(s_oper, &gen_r_cfg, &s_gen_r));
    RETURN_IF_ERR(mcpwm_new_generator(s_oper, &gen_l_cfg, &s_gen_l));

    RETURN_IF_ERR(configure_wave(s_gen_r, s_cmp_r));
    RETURN_IF_ERR(configure_wave(s_gen_l, s_cmp_l));

    // Avoid compare=0 edge ambiguity. Outputs remain forced LOW at start.
    RETURN_IF_ERR(mcpwm_comparator_set_compare_value(s_cmp_r, 1));
    RETURN_IF_ERR(mcpwm_comparator_set_compare_value(s_cmp_l, 1));
    RETURN_IF_ERR(mcpwm_generator_set_force_level(s_gen_r, 0, true));
    RETURN_IF_ERR(mcpwm_generator_set_force_level(s_gen_l, 0, true));

    RETURN_IF_ERR(mcpwm_timer_enable(s_timer));
    RETURN_IF_ERR(mcpwm_timer_start_stop(s_timer, MCPWM_TIMER_START_NO_STOP));

    s_current_percent = 0;
    s_ready = true;
    ESP_LOGI(TAG, "Motor MCPWM pronto: %d Hz, %d ticks", 
             PWM_RESOLUTION_HZ / PWM_PERIOD_TICKS, PWM_PERIOD_TICKS);
    return ESP_OK;
}

void bts7960_coast(void)
{
    if (!s_ready) return;
    // Disable bridge BEFORE changing PWM outputs.
    ESP_ERROR_CHECK(gpio_set_level(PIN_REN, 0));
    ESP_ERROR_CHECK(gpio_set_level(PIN_LEN, 0));
    force_both_low();
    s_current_percent = 0;
}

void bts7960_brake(void)
{
    if (!s_ready) return;
    force_both_low();
    ESP_ERROR_CHECK(gpio_set_level(PIN_REN, 1));
    ESP_ERROR_CHECK(gpio_set_level(PIN_LEN, 1));
    s_current_percent = 0;
}

esp_err_t bts7960_set_speed(int percent)
{
    if (!s_ready) return ESP_ERR_INVALID_STATE;
    if (percent < -100 || percent > 100) return ESP_ERR_INVALID_ARG;

    if (percent == 0) {
        bts7960_coast();
        return ESP_OK;
    }
    // Disallow instantaneous reverse while previous command is nonzero.
    if ((s_current_percent > 0 && percent < 0) ||
        (s_current_percent < 0 && percent > 0)) {
        bts7960_coast();
        return ESP_ERR_INVALID_STATE;
    }

    // Both signals LOW, bridge disabled while reconfiguring direction/duty.
    bts7960_coast();

    const unsigned magnitude = (unsigned)(percent < 0 ? -percent : percent);
    const unsigned compare = magnitude * PWM_PERIOD_TICKS / 100;
    mcpwm_gen_handle_t selected_gen = percent > 0 ? s_gen_r : s_gen_l;
    mcpwm_cmpr_handle_t selected_cmp = percent > 0 ? s_cmp_r : s_cmp_l;

    if (compare == PWM_PERIOD_TICKS) {
        // Exact 100%: constant HIGH on only the selected pin.
        RETURN_IF_ERR(mcpwm_generator_set_force_level(selected_gen, 1, true));
    } else {
        RETURN_IF_ERR(mcpwm_comparator_set_compare_value(selected_cmp, compare));
        RETURN_IF_ERR(mcpwm_generator_set_force_level(selected_gen, -1, true));
    }

    RETURN_IF_ERR(gpio_set_level(PIN_REN, 1));
    RETURN_IF_ERR(gpio_set_level(PIN_LEN, 1));
    s_current_percent = percent;
    return ESP_OK;
}
