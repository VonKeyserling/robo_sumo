#include "steering.h"
#include "servo.h"
#include <stdbool.h>
#include <stdint.h>

// SEGURANCA: teste com a articulacao de direcao DESACOPLADA.
// Este algoritmo NAO tem realimentacao de posicao.
#define STEER_DEADZONE 2500
#define STEER_MAX_TARGET_DEG                                                   \
  90.0f // +/- 90 graus VIRTUAIS a partir da posicao no boot
#define STEER_RATE_DEG_S_AT_100 300.0f // CALIBRAR NA PRATICA
#define STEER_MAX_DRIVE_PERCENT 25
#define STEER_MIN_DRIVE_PERCENT 12
#define STEER_KP_PERCENT_PER_DEG 1.2f
#define STEER_STOP_TOLERANCE_DEG 1.5f
#define STEER_SERVO_INVERT 0 // 1 se o servo girar invertido
#define STEER_MAX_DT_US 100000LL

static struct {
  bool initialized;
  float estimated_deg;
  int logical_speed_percent; // comando logico, antes da inversao eletrica
  int64_t last_us;
} s;

static float clampf_local(float n, float low, float high) {
  if (n < low)
    return low;
  if (n > high)
    return high;
  return n;
}

static esp_err_t apply_logical_speed(int percent) {
  if (percent == s.logical_speed_percent)
    return ESP_OK;
#if STEER_SERVO_INVERT
  int physical = -percent;
#else
  int physical = percent;
#endif
  esp_err_t err =
      (physical == 0) ? servo360_stop() : servo360_set_speed(physical);
  if (err == ESP_OK)
    s.logical_speed_percent = percent;
  return err;
}

esp_err_t steering360_init(int servo_gpio) {
  if (s.initialized)
    return ESP_ERR_INVALID_STATE;
  esp_err_t err = servo360_init(servo_gpio);
  if (err != ESP_OK)
    return err;
  err = servo360_stop();
  if (err != ESP_OK)
    return err;
  // PONTO ZERO VIRTUAL: a posicao FISICA atual no boot passa a ser 0.
  // Nao existe medicao de angulo; 90 graus sao apenas estimados.
  s.estimated_deg = 0;
  s.logical_speed_percent = 0;
  s.last_us = 0;
  s.initialized = true;
  return ESP_OK;
}

esp_err_t steering360_update(int16_t lx, int64_t now_us) {
  if (!s.initialized)
    return ESP_ERR_INVALID_STATE;
  // Primeiro estimamos o movimento realizado desde a ultima atualizacao.
  if (s.last_us > 0) {
    int64_t elapsed_us = now_us - s.last_us;
    if (elapsed_us < 0)
      elapsed_us = 0;
    if (elapsed_us > STEER_MAX_DT_US) {
      s.last_us = now_us;
      return apply_logical_speed(0); // ciclo muito atrasado: parar
    }
    const float dt = (float)elapsed_us / 1000000.0f;
    s.estimated_deg += (float)s.logical_speed_percent *
                       (STEER_RATE_DEG_S_AT_100 / 100.0f) * dt;
    s.estimated_deg = clampf_local(s.estimated_deg, -STEER_MAX_TARGET_DEG,
                                   STEER_MAX_TARGET_DEG);
  }
  s.last_us = now_us;

  int32_t x = lx;
  if (x > -STEER_DEADZONE && x < STEER_DEADZONE)
    x = 0;
  float target_deg = (float)x * (STEER_MAX_TARGET_DEG / 32767.0f);
  target_deg =
      clampf_local(target_deg, -STEER_MAX_TARGET_DEG, STEER_MAX_TARGET_DEG);
  float error_deg = target_deg - s.estimated_deg;
  if (error_deg > -STEER_STOP_TOLERANCE_DEG &&
      error_deg < STEER_STOP_TOLERANCE_DEG)
    return apply_logical_speed(0);

  int speed = (int)(STEER_KP_PERCENT_PER_DEG *
                        (error_deg < 0 ? -error_deg : error_deg) +
                    0.5f);
  if (speed < STEER_MIN_DRIVE_PERCENT)
    speed = STEER_MIN_DRIVE_PERCENT;
  if (speed > STEER_MAX_DRIVE_PERCENT)
    speed = STEER_MAX_DRIVE_PERCENT;
  if (error_deg < 0)
    speed = -speed;

  // Protecao contra avancar ainda mais alem do LIMITE ESTIMADO.
  if ((s.estimated_deg >= STEER_MAX_TARGET_DEG && speed > 0) ||
      (s.estimated_deg <= -STEER_MAX_TARGET_DEG && speed < 0))
    speed = 0;
  return apply_logical_speed(speed);
}

esp_err_t steering360_stop(void) {
  if (!s.initialized)
    return ESP_ERR_INVALID_STATE;
  s.last_us = 0; // nao integre tempo em que a comunicacao esteve perdida
  return apply_logical_speed(0);
}

float steering360_estimated_deg(void) { return s.estimated_deg; }
int steering360_last_speed_percent(void) { return s.logical_speed_percent; }
