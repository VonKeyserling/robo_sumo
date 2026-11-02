
#include <stdbool.h>
#include <stdint.h>

#include "esp_log.h"
#include "esp_timer.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "nvs_flash.h"

#include "bts7960.h"
#include "cyclone2.h"
#include "steering.h"

#define GAMESIR_MAC "D0:56:80:C9:3B:E1"

// Servo continuo 360: controla velocidade e ESTIMA posicao pelo tempo.
// Antes do boot, posicione fisicamente a direcao no centro.
#define SERVO_GPIO 32

// Deixe 0 para testar o controle sem acionar o motor.
#define MOTOR_TEST_ENABLED 1

// Potencia maxima do motor. Comece com 30 em testes com rodas suspensas.
#define MOTOR_MAX_POWER 100

#define TRIGGER_DEADZONE 10
// 300 ms em roda livre antes de inverter o sentido, sem travar a task.
#define MOTOR_REVERSAL_PAUSE_US 300000LL
// Rampa de 3% por iteracao (20 ms), reduzindo solavancos.
#define MOTOR_RAMP_STEP 3

static const char *TAG = "ROBO_SUMO";

static void gamesir_callback(const cyclone2_event_t *event, void *arg) {
  (void)arg;

  switch (event->type) {

  case CYCLONE2_CONNECTED:
    ESP_LOGI(TAG, "GameSir conectado!");
    break;

  case CYCLONE2_DISCONNECTED:
    ESP_LOGW(TAG, "GameSir desconectado!");
    // O loop principal desabilita a ponte
    // quando a leitura HID deixa de ser válida.
    break;

  case CYCLONE2_INPUT:
    if (!event->gamepad.decoded) {
      static unsigned count = 0;

      if ((count++ % 100) == 0) {
        ESP_LOGW(TAG, "HID desconhecido: ID=0x%02X LEN=%u",
                 (unsigned)event->report_id, (unsigned)event->length);

        if (event->data && event->length) {
          size_t n = event->length;
          if (n > 32)
            n = 32;
          ESP_LOG_BUFFER_HEX(TAG, event->data, n);
        }
      }
    }
    break;

  default:
    break;
  }
}

// Converte o gatilho de 0..255 em 0..MOTOR_MAX_POWER.
static int trigger_to_power(uint8_t trigger) {
  if (trigger <= TRIGGER_DEADZONE) {
    return 0;
  }

  return ((int)(trigger - TRIGGER_DEADZONE) * MOTOR_MAX_POWER) /
         (255 - TRIGGER_DEADZONE);
}

// Controla a ponte por um comando ASSINADO: + frente, - re, 0 livre.
// Uma inversao em movimento faz coast e aguarda 300 ms antes de dar potencia.
// A rampa limita a variacao do PWM e a task principal continua responsiva.
typedef struct {
  int applied_percent;
  int last_direction; // -1 re, +1 frente, 0 ainda nao movimentou
  int64_t reverse_ready_at_us;
} motor_motion_t;

static void motor_stop(motor_motion_t *motion) {
  const int64_t now_us = esp_timer_get_time();
  if (motion->applied_percent != 0) {
    motion->last_direction = motion->applied_percent > 0 ? 1 : -1;
    // Mantem esta janela mesmo se ambos gatilhos forem soltos por 1 ciclo.
    motion->reverse_ready_at_us = now_us + MOTOR_REVERSAL_PAUSE_US;
  }
  bts7960_coast();
  motion->applied_percent = 0;
}

static esp_err_t motor_update(motor_motion_t *motion, int requested_percent) {
  const int64_t now_us = esp_timer_get_time();

  if (requested_percent == 0) {
    if (motion->applied_percent != 0)
      motor_stop(motion);
    return ESP_OK;
  }

  const int requested_direction = requested_percent > 0 ? 1 : -1;

  // Inversao durante movimento: primeiro desabilita a ponte.
  if (motion->applied_percent != 0 &&
      motion->last_direction != requested_direction) {
    motor_stop(motion);
    return ESP_OK;
  }

  // Protecao tambem para frente -> solta 50 ms -> re.
  if (motion->applied_percent == 0 && motion->last_direction != 0 &&
      motion->last_direction != requested_direction &&
      now_us < motion->reverse_ready_at_us) {
    return ESP_OK;
  }

  // Acelere e desacelere suavemente, inclusive na re.
  int delta = requested_percent - motion->applied_percent;
  if (delta > MOTOR_RAMP_STEP)
    delta = MOTOR_RAMP_STEP;
  if (delta < -MOTOR_RAMP_STEP)
    delta = -MOTOR_RAMP_STEP;
  int next = motion->applied_percent + delta;

  if (next != motion->applied_percent) {
    esp_err_t err = bts7960_set_speed(next);
    if (err != ESP_OK) {
      motor_stop(motion); // falha de PWM => desabilita a ponte
      return err;
    }
    motion->applied_percent = next;
    motion->last_direction = next > 0 ? 1 : -1;
  }
  return ESP_OK;
}

void app_main(void) {
  esp_err_t err = nvs_flash_init();

  if (err == ESP_ERR_NVS_NO_FREE_PAGES ||
      err == ESP_ERR_NVS_NEW_VERSION_FOUND) {
    ESP_ERROR_CHECK(nvs_flash_erase());
    err = nvs_flash_init();
  }

  ESP_ERROR_CHECK(err);

  // O centro inicial e uma referencia VIRTUAL, nao medido.
  // Alinhe o eixo da direcao fisicamente antes de iniciar.
  ESP_LOGI(TAG, "Inicializando direcao 360 com estimativa de posicao...");
  ESP_ERROR_CHECK(steering360_init(SERVO_GPIO));

  // Inicializa o driver BTS7960 em estado seguro
  ESP_LOGI(TAG, "Inicializando BTS7960...");
  ESP_ERROR_CHECK(bts7960_init());
  bts7960_coast();

  // Inicializa GameSir Cyclone 2
  ESP_LOGI(TAG, "Inicializando GameSir...");
  ESP_ERROR_CHECK(cyclone2_start(GAMESIR_MAC, gamesir_callback, NULL));

  cyclone2_gamepad_t gamepad;
  motor_motion_t motor = {0};
  unsigned log_count = 0;

  while (1) {

    if (cyclone2_get_gamepad(&gamepad)) {

      // 1. Analógico LX => angulo-alvo ESTIMADO.
      // Se soltar o LX, o alvo volta a 0 e o controle tenta
      // girar no sentido oposto ate o centro estimado.
      esp_err_t steering_err =
          steering360_update(gamepad.lx, esp_timer_get_time());
      if (steering_err != ESP_OK) {
        ESP_LOGE(TAG, "Falha ao atualizar servo: %s",
                 esp_err_to_name(steering_err));
        motor_stop(&motor);
        (void)steering360_stop();
        vTaskDelay(pdMS_TO_TICKS(20));
        continue;
      }

      // 2. RT -> frente (+), LT -> re (-), ambos -> parar.
      const uint8_t rt = gamepad.rt;
      const uint8_t lt = gamepad.lt;
      const int forward_power = trigger_to_power(rt);
      const int reverse_power = trigger_to_power(lt);

      int requested_percent = 0;
      if (forward_power > 0 && reverse_power == 0) {
        requested_percent = forward_power;
      } else if (reverse_power > 0 && forward_power == 0) {
        requested_percent = -reverse_power;
      } // ambos apertados ou soltos: motor em roda livre

#if MOTOR_TEST_ENABLED
      ESP_ERROR_CHECK(motor_update(&motor, requested_percent));
#else
      motor_stop(&motor);
#endif

      if (++log_count >= 25) {
        log_count = 0;

        ESP_LOGI(TAG,
                 "LX:%d CentroEstimado:%.1fdeg ServoCmd:%+d%% "
                 "RT:%u LT:%u Pedido:%+d%% Motor:%+d%%",
                 gamepad.lx, (double)steering360_estimated_deg(),
                 steering360_last_speed_percent(), rt, lt, requested_percent,
                 motor.applied_percent);
      }

    } else {
      // Falha de comunicação / pacote inválido
      motor_stop(&motor);

      // Com RF perdido nao tente retornar ao centro sem sensor.
      // Apenas interrompa o giro.
      (void)steering360_stop();
    }

    vTaskDelay(pdMS_TO_TICKS(20));
  }
}
