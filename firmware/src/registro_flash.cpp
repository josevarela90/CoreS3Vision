#include "registro_flash.h"
#include <stdio.h>
#include <stdlib.h>
#include <sys/stat.h>
#include <dirent.h>
#include <inttypes.h>
#include <string.h>
#include "esp_log.h"
#include "esp_timer.h"
#include "img_converters.h"

static const char *TAG = "REGISTRO_FLASH";
static const char *DIR_CAPTURAS = "/storage";
static const char *CSV_EVENTOS = "/storage/eventos.csv";
static const int MAX_CAPTURAS = 25;
static bool listo = false;

static int contar_capturas() {
    DIR *d = opendir(DIR_CAPTURAS);
    if (!d) return 0;
    int n = 0;
    while (auto *e = readdir(d)) {
        if (strstr(e->d_name, ".jpg")) n++;
    }
    closedir(d);
    return n;
}

static bool borrar_mas_antigua() {
    DIR *d = opendir(DIR_CAPTURAS);
    if (!d) return false;
    char oldest[128] = {};
    while (auto *e = readdir(d)) {
        if (!strstr(e->d_name, ".jpg")) continue;
        if (oldest[0] == 0 || strcmp(e->d_name, oldest) < 0) {
            strlcpy(oldest, e->d_name, sizeof(oldest));
        }
    }
    closedir(d);
    if (oldest[0] == 0) return false;
    char path[220];
    snprintf(path, sizeof(path), "%s/%s", DIR_CAPTURAS, oldest);
    if (remove(path) == 0) {
        ESP_LOGI(TAG, "Eliminada captura antigua: %s", oldest);
        return true;
    }
    return false;
}

bool registro_flash_iniciar() {
    mkdir(DIR_CAPTURAS, 0775);
    struct stat st = {};
    if (stat(CSV_EVENTOS, &st) != 0) {
        FILE *f = fopen(CSV_EVENTOS, "w");
        if (!f) return false;
        fprintf(f, "id,martillos,pinzas,respuesta_ms,conf_martillo,conf_pinza,imagen\n");
        fclose(f);
    }
    listo = true;
    ESP_LOGI(TAG, "Registro interno listo");
    return true;
}

bool registro_flash_guardar_evento(const uint8_t *rgb565, int ancho, int alto, uint8_t calidad_jpeg, int martillos, int pinzas, uint64_t respuesta_ms, float conf_martillo, float conf_pinza) {
    if (!listo || !rgb565 || (martillos <= 0 && pinzas <= 0)) return false;
    while (contar_capturas() >= MAX_CAPTURAS) {
        if (!borrar_mas_antigua()) break;
    }
    uint64_t id = (uint64_t)(esp_timer_get_time() / 1000ULL);
    char name[96];
    snprintf(name, sizeof(name), "captura_%010" PRIu64 ".jpg", id);
    char path[220];
    snprintf(path, sizeof(path), "%s/%s", DIR_CAPTURAS, name);
    uint8_t *jpg = nullptr;
    size_t jpg_len = 0;
    size_t bytes = (size_t)ancho * (size_t)alto * 2U;
    if (!fmt2jpg((uint8_t*)rgb565, bytes, ancho, alto, PIXFORMAT_RGB565, calidad_jpeg, &jpg, &jpg_len)) return false;
    FILE *img = fopen(path, "wb");
    if (!img) { free(jpg); return false; }
    fwrite(jpg, 1, jpg_len, img);
    fclose(img);
    free(jpg);
    FILE *csv = fopen(CSV_EVENTOS, "a");
    if (!csv) return false;
    fprintf(csv, "%" PRIu64 ",%d,%d,%" PRIu64 ",%.3f,%.3f,%s\n", id, martillos, pinzas, respuesta_ms, conf_martillo, conf_pinza, name);
    fclose(csv);
    ESP_LOGI(TAG, "Guardado %s | M=%d P=%d | %" PRIu64 " ms", name, martillos, pinzas, respuesta_ms);
    return true;
}

