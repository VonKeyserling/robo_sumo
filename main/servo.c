#include "servo.h"
#include "driver/mcpwm_cmpr.h"
#include "driver/mcpwm_gen.h"
#include "driver/mcpwm_oper.h"
#include "driver/mcpwm_timer.h"
#include "driver/mcpwm_types.h"
#include "esp_log.h"
#include <stdint.h>

#define SERVO_MIN_PULSEWIDTH_US 1000 // Largura para 0 graus
#define SERVO_MAX_PULSEWIDTH_US 2000 // Largura para 180 graus
#define SERVO_MIN_DEGREE 0
#define SERVO_MAX_DEGREE 180

#define SERVO_PULSE_GPIO 18
#define SERVO_TIMEBASE_RESOLUTION_HZ 1000000 // 1 tick = 1 us
#define SERVO_TIMEBASE_PERIOD 20000          // 20 ms = 50 Hz

static mcpwm_cmpr_handle_t comparator_handler = NULL;

static inline uint32_t get_angle(int angle) {
  if (angle < SERVO_MIN_DEGREE)
    angle = SERVO_MIN_DEGREE;
  if (angle > SERVO_MAX_DEGREE)
    angle = SERVO_MAX_DEGREE;

  return SERVO_MIN_PULSEWIDTH_US +
         ((uint32_t)angle *
          (SERVO_MAX_PULSEWIDTH_US - SERVO_MIN_PULSEWIDTH_US)) /
             SERVO_MAX_DEGREE;
}

void init_servo(void) {
  ESP_LOGI(TAG, "Inicializando Timer e Operator do MCPWM");
  mcpwm_timer_handle_t timer = NULL;
  mcpwm_timer_config_t timer_config = {
      .group_id = 0,
      .clk_src = MCPWM_TIMER_CLK_SRC_DEFAULT,
      .count_mode = MCPWM_TIMER_COUNT_MODE_UP,
      .period_ticks = SERVO_TIMEBASE_PERIOD,
      .resolution_hz = SERVO_TIMEBASE_RESOLUTION_HZ,
  };

  ESP_ERROR_CHECK(mcpwm_new_timer(&timer_config, &timer));

  mcpwm_oper_handle_t operator_handler = NULL;
  mcpwm_operator_config_t operator_config = {.group_id = 0};
  ESP_ERROR_CHECK(mcpwm_new_operator(&operator_config, &operator_handler));
  ESP_ERROR_CHECK(mcpwm_operator_connect_timer(operator_handler, timer));

  mcpwm_comparator_config_t comparator_config = {
      .flags = {.update_cmp_on_tez = true},
  };
  ESP_ERROR_CHECK(mcpwm_new_comparator(operator_handler, &comparator_config,
                                       &comparator_handler));

  mcpwm_gen_handle_t generator_handler = NULL;
  mcpwm_generator_config_t generator_config = {.gen_gpio_num =
                                                   SERVO_PULSE_GPIO};
  ESP_ERROR_CHECK(mcpwm_new_generator(operator_handler, &generator_config,
                                      &generator_handler));

  ESP_ERROR_CHECK(
      mcpwm_comparator_set_compare_value(comparator_handler, get_angle(90)));

  ESP_ERROR_CHECK(mcpwm_generator_set_action_on_timer_event(
      generator_handler, MCPWM_GEN_TIMER_EVENT_ACTION(MCPWM_TIMER_DIRECTION_UP,
                                                      MCPWM_TIMER_EVENT_EMPTY,
                                                      MCPWM_GEN_ACTION_HIGH)));

  ESP_ERROR_CHECK(mcpwm_generator_set_action_on_compare_event(
      generator_handler,
      MCPWM_GEN_COMPARE_EVENT_ACTION(
          MCPWM_TIMER_DIRECTION_UP, comparator_handler, MCPWM_GEN_ACTION_LOW)));

  ESP_ERROR_CHECK(mcpwm_timer_enable(timer));
  ESP_ERROR_CHECK(mcpwm_timer_start_stop(timer, MCPWM_TIMER_START_NO_STOP));
}

void set_servo_angle(int angle) {
  uint32_t compare_val = get_angle(angle);
  ESP_LOGI(TAG, "Angulo: %d, Pulso: %lu us", angle, compare_val);
  ESP_ERROR_CHECK(
      mcpwm_comparator_set_compare_value(comparator_handler, compare_val));
}
