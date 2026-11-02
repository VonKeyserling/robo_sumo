#pragma once
#include "esp_err.h"
#include <stdint.h>

// Driver de direcao ABERTA: sem sensor de angulo, a posicao e ESTIMADA.
// O eixo deve estar alinhado fisicamente ao centro na inicializacao.
esp_err_t steering360_init(int servo_gpio);
esp_err_t steering360_update(int16_t lx, int64_t now_us);
esp_err_t
steering360_stop(void); // falha de comunicacao: para, nao busca centro
float steering360_estimated_deg(void);
int steering360_last_speed_percent(void);
