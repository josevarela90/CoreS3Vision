#include "botones.h"

void botones_init()
{
    gpio_config_t config = {};

    config.pin_bit_mask =
        (1ULL << BOTON_ROJO_GPIO) |
        (1ULL << BOTON_AZUL_GPIO);

    config.mode = GPIO_MODE_INPUT;
    config.pull_up_en = GPIO_PULLUP_ENABLE;
    config.pull_down_en = GPIO_PULLDOWN_DISABLE;
    config.intr_type = GPIO_INTR_DISABLE;

    gpio_config(&config);
}

bool boton_rojo_pulsado()
{
    return gpio_get_level(BOTON_ROJO_GPIO) == 0;
}

bool boton_azul_pulsado()
{
    return gpio_get_level(BOTON_AZUL_GPIO) == 0;
}
