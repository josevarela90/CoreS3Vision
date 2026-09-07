#pragma once
#include <stdint.h>
bool registro_flash_iniciar();
bool registro_flash_guardar_evento(const uint8_t *rgb565, int ancho, int alto, uint8_t calidad_jpeg, int martillos, int pinzas, uint64_t respuesta_ms, float conf_martillo, float conf_pinza);
