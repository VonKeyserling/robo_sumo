#pragma once

#include "esp_err.h"

// ESP32 original / BTS7960 IBT-2.
// Requer uma chamada a bts7960_init() antes das outras funcoes.
esp_err_t bts7960_init(void);

// Velocidade de -100 a +100 (%). Zero = coast (ponte desabilitada).
// Inversao brusca do sentido (sem passar por 0) e bloqueada.
esp_err_t bts7960_set_speed(int percent);

// Forca R_EN=L_EN=0: saidas da ponte desabilitadas (motor em roda livre).
void bts7960_coast(void);

// Forca RPWM=LPWM=0 com enables ativos: frenagem dinamica (motor curto-circuitado).
void bts7960_brake(void);
