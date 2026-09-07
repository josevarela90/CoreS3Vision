#pragma once

#include "driver/gpio.h"

static constexpr gpio_num_t BOTON_ROJO_GPIO = GPIO_NUM_2;
static constexpr gpio_num_t BOTON_AZUL_GPIO = GPIO_NUM_1;

void botones_init();
bool boton_rojo_pulsado();
bool boton_azul_pulsado();
