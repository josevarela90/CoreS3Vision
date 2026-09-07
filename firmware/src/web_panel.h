#pragma once
#include <stdint.h>

bool web_panel_iniciar();
bool web_panel_consumir_captura();

void web_panel_actualizar_resultado(
    int martillos,
    int pinzas,
    uint64_t respuesta_ms,
    float confianza_martillo,
    float confianza_pinza
);

bool web_panel_iniciar_wifi_temprano();
bool web_panel_iniciar_http_tarde();

bool web_panel_diagnostico_wifi();
