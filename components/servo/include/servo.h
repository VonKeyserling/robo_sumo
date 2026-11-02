#pragma once

#include "esp_err.h"
#include <stdbool.h>
#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

/*
 * TowerPro MG90S 360 graus, versao de ROTACAO CONTINUA.
 * Controle por pulso de 50 Hz (periodo de 20 ms).
 * Valores iniciais TIPICOS, calibrar no servo real.
 *
 * 1000 us: giro em um sentido
 * 1500 us: parada (neutro)
 * 2000 us: giro no sentido contrario
 *
 * A largura de pulso NAO define angulo em servo de rotacao continua.
 */
#define SERVO360_DEFAULT_GPIO 32
#define SERVO360_PERIOD_US 20000U
#define SERVO360_MIN_PULSE_US 1000U
#define SERVO360_DEFAULT_NEUTRAL_US 1500U
#define SERVO360_MAX_PULSE_US 2000U

/* Inicializa o MCPWM no GPIO escolhido. Chamar uma unica vez. */
esp_err_t servo360_init(int gpio_num);

/* Velocidade solicitada de -100 a +100% (saturada); zero envia neutro. */
esp_err_t servo360_set_speed(int speed_percent);

/* Envia o neutro para parar a ROTACAO; nao retorna a um angulo fisico. */
esp_err_t servo360_stop(void);

/* Calibra o neutro de parada: aceita 1400..1600 us e para o servo. */
esp_err_t servo360_set_neutral_us(uint16_t neutral_us);

/* Envia pulso bruto para diagnostico: aceita 1000..2000 us. */
esp_err_t servo360_set_pulse_us(uint16_t pulse_us);

/* Consulta se a inicializacao foi concluida. */
bool servo360_is_initialized(void);

#ifdef __cplusplus
}
#endif
