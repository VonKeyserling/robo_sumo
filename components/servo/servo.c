#include "servo.h"
#include <stddef.h>

#include "driver/mcpwm_cmpr.h"
#include "driver/mcpwm_gen.h"
#include "driver/mcpwm_oper.h"
#include "driver/mcpwm_timer.h"
#include "driver/mcpwm_types.h"
#include "esp_log.h"

#define SERVO360_TICKS_PER_SECOND 1000000U // 1 tick = 1 us
#define SERVO360_MCPWM_GROUP_ID                                                \
  1                          // ESP32 original: reservar grupo 0 para motores
#define SERVO360_SPAN_US 500 // +/-500 us a partir do neutro

static const char *TAG = "MG90S360";
static mcpwm_cmpr_handle_t s_comparator = NULL;
static uint16_t s_neutral_us = SERVO360_DEFAULT_NEUTRAL_US;
static bool s_initialized = false;

bool servo360_is_initialized(void) { return s_initialized; }

esp_err_t servo360_init(int gpio_num) {
  if (s_initialized) {
    return ESP_ERR_INVALID_STATE;
  }
  if (gpio_num < 0) {
    return ESP_ERR_INVALID_ARG;
  }

  mcpwm_timer_handle_t timer = NULL;
  mcpwm_oper_handle_t oper = NULL;
  mcpwm_cmpr_handle_t cmp = NULL;
  mcpwm_gen_handle_t gen = NULL;
  bool timer_enabled = false;
  esp_err_t err;

  const mcpwm_timer_config_t timer_cfg = {
      .group_id = SERVO360_MCPWM_GROUP_ID,
      .clk_src = MCPWM_TIMER_CLK_SRC_DEFAULT,
      .resolution_hz = SERVO360_TICKS_PER_SECOND,
      .period_ticks = SERVO360_PERIOD_US,
      .count_mode = MCPWM_TIMER_COUNT_MODE_UP,
  };
  err = mcpwm_new_timer(&timer_cfg, &timer);
  if (err != ESP_OK)
    goto fail;

  const mcpwm_operator_config_t oper_cfg = {
      .group_id = SERVO360_MCPWM_GROUP_ID,
  };
  err = mcpwm_new_operator(&oper_cfg, &oper);
  if (err != ESP_OK)
    goto fail;

  err = mcpwm_operator_connect_timer(oper, timer);
  if (err != ESP_OK)
    goto fail;

  const mcpwm_comparator_config_t cmp_cfg = {
      .flags.update_cmp_on_tez = true,
  };
  err = mcpwm_new_comparator(oper, &cmp_cfg, &cmp);
  if (err != ESP_OK)
    goto fail;

  const mcpwm_generator_config_t gen_cfg = {
      .gen_gpio_num = gpio_num,
  };
  err = mcpwm_new_generator(oper, &gen_cfg, &gen);
  if (err != ESP_OK)
    goto fail;

  /* CRITICO: iniciar com pulso neutro ANTES de ativar o timer. */
  err = mcpwm_comparator_set_compare_value(cmp, s_neutral_us);
  if (err != ESP_OK)
    goto fail;

  /* A cada 20 ms: sobe sinal e desce depois de s_neutral_us. */
  err = mcpwm_generator_set_action_on_timer_event(
      gen, MCPWM_GEN_TIMER_EVENT_ACTION(MCPWM_TIMER_DIRECTION_UP,
                                        MCPWM_TIMER_EVENT_EMPTY,
                                        MCPWM_GEN_ACTION_HIGH));
  if (err != ESP_OK)
    goto fail;

  err = mcpwm_generator_set_action_on_compare_event(
      gen, MCPWM_GEN_COMPARE_EVENT_ACTION(MCPWM_TIMER_DIRECTION_UP, cmp,
                                          MCPWM_GEN_ACTION_LOW));
  if (err != ESP_OK)
    goto fail;

  err = mcpwm_timer_enable(timer);
  if (err != ESP_OK)
    goto fail;
  timer_enabled = true;

  err = mcpwm_timer_start_stop(timer, MCPWM_TIMER_START_NO_STOP);
  if (err != ESP_OK)
    goto fail;

  s_comparator = cmp;
  s_initialized = true;
  /* Timer, operador e gerador continuam ativos durante a vida do driver. */
  ESP_LOGI(TAG, "GPIO %d | 50 Hz | neutro %u us", gpio_num,
           (unsigned)s_neutral_us);
  return ESP_OK;

fail:
  /* Melhor esforco para liberar recursos caso inicializacao falhe. */
  if (timer_enabled)
    (void)mcpwm_timer_disable(timer);
  if (gen)
    (void)mcpwm_del_generator(gen);
  if (cmp)
    (void)mcpwm_del_comparator(cmp);
  if (oper)
    (void)mcpwm_del_operator(oper);
  if (timer)
    (void)mcpwm_del_timer(timer);
  ESP_LOGE(TAG, "Erro na inicializacao MCPWM: %s", esp_err_to_name(err));
  return err;
}

esp_err_t servo360_set_pulse_us(uint16_t pulse_us) {
  if (!s_initialized) {
    return ESP_ERR_INVALID_STATE;
  }
  if (pulse_us < SERVO360_MIN_PULSE_US || pulse_us > SERVO360_MAX_PULSE_US) {
    return ESP_ERR_INVALID_ARG;
  }
  return mcpwm_comparator_set_compare_value(s_comparator, pulse_us);
}

esp_err_t servo360_set_speed(int speed_percent) {
  if (!s_initialized) {
    return ESP_ERR_INVALID_STATE;
  }
  if (speed_percent > 100)
    speed_percent = 100;
  if (speed_percent < -100)
    speed_percent = -100;

  int32_t pulse =
      (int32_t)s_neutral_us + (int32_t)speed_percent * SERVO360_SPAN_US / 100;
  if (pulse < (int32_t)SERVO360_MIN_PULSE_US)
    pulse = SERVO360_MIN_PULSE_US;
  if (pulse > (int32_t)SERVO360_MAX_PULSE_US)
    pulse = SERVO360_MAX_PULSE_US;

  return servo360_set_pulse_us((uint16_t)pulse);
}

esp_err_t servo360_stop(void) { return servo360_set_speed(0); }

esp_err_t servo360_set_neutral_us(uint16_t neutral_us) {
  if (!s_initialized) {
    return ESP_ERR_INVALID_STATE;
  }
  if (neutral_us < 1400U || neutral_us > 1600U) {
    return ESP_ERR_INVALID_ARG;
  }
  /* Faz o servo parar primeiro, com o novo neutro. */
  esp_err_t err = servo360_set_pulse_us(neutral_us);
  if (err == ESP_OK) {
    s_neutral_us = neutral_us;
  }
  return err;
}
