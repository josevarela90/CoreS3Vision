#include <stdint.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <inttypes.h>
#include <sys/stat.h>

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/semphr.h"

#include "driver/i2c_master.h"
#include "esp_private/i2c_platform.h"
#include "esp_camera.h"
#include "img_converters.h"
#include <dirent.h>
#include "esp_codec_dev.h"
#include "esp_heap_caps.h"
#include "esp_log.h"
#include "esp_partition.h"
#include "esp_spiffs.h"
#include "esp_timer.h"

#include "bsp/esp-bsp.h"
#include "lvgl.h"

#include "yolo26.hpp"
#include "voz_martillo.h"
#include "botones.h"
#include "web_panel.h"
#include "registro_flash.h"

static const char *TAG = "HAMMER_AUTO";

// ============================================================
// CONFIGURACION GENERAL
// ============================================================

// Confianza minima del nuevo modelo.
static constexpr int FOTO_ANCHO = 320;
static constexpr int FOTO_ALTO = 240;
static constexpr int MODELO_ANCHO = 256;
static constexpr int MODELO_ALTO = 256;

// Video.
static constexpr int INTERVALO_VIDEO_MS = 100;

// Cada cuanto intentamos una nueva inferencia.
// La inferencia puede tardar; entre inferencias vuelve el video.
static constexpr uint64_t INTERVALO_DETECCION_MS = 1000;

// Historial.
static constexpr int MAX_HISTORIAL = 200;
static constexpr int MAX_RASTREOS = 16;
static constexpr int MAX_NUEVOS_POR_FRAME = 10;

// Un rastreo se conserva varios segundos para que un martillo
// no vuelva a contarse por una oclusion o movimiento breve.
static constexpr uint64_t RASTREO_EXPIRA_MS = 8000;

// Criterios para seguir el mismo objeto entre frames.
static constexpr float IOU_MINIMA_RASTREO = 0.18f;
static constexpr int DISTANCIA_CENTRO_MAX = 70;

// Criterios para reconocer visualmente un martillo ya guardado.
// Son deliberadamente conservadores.
static constexpr int HAMMING_FIRMA_MAX = 10;
static constexpr int DIFERENCIA_COLOR_MAX = 120;
static constexpr int DIFERENCIA_ASPECTO_MAX = 400;

// JPEG del historial.
static constexpr uint8_t CALIDAD_JPEG = 72;

// SPIFFS.
static constexpr const char *ETIQUETA_STORAGE = "storage";
static constexpr const char *BASE_STORAGE = "/storage";
static constexpr const char *ARCHIVO_HISTORIAL =
    "/storage/historial.csv";

// Clases del modelo YOLO26 personalizado.
// IMPORTANTE: deben conservar el mismo orden usado durante el entrenamiento.
static const char *CLASES_HERRAMIENTAS[] = {
    "hammer",
    "plier"
};

static constexpr int NUM_CLASES_HERRAMIENTAS = 2;

static constexpr float CONFIANZA_HAMMER = 0.10f;
static constexpr float CONFIANZA_PLIER  = 0.10f;
static constexpr float CONFIANZA_ENTRADA_YOLO = 0.10f;
static constexpr float PLIER_RESCATE_MIN = 0.15f;
static constexpr float HAMMER_MAX_EN_CONFLICTO = 0.90f;
static constexpr float IOU_CONFLICTO_CLASES = 0.30f;

// ============================================================
// AXP2101 / POWER
// ============================================================

static constexpr uint8_t AXP2101_DIRECCION = 0x34;
static constexpr uint8_t AXP2101_IRQ_HABILITAR_1 = 0x41;
static constexpr uint8_t AXP2101_IRQ_ESTADO_1 = 0x49;

static constexpr uint8_t AXP2101_EVENTO_LARGO = 0x01;
static constexpr uint8_t AXP2101_EVENTO_CORTO = 0x02;
static constexpr uint8_t AXP2101_MASCARA_POWER = 0x0C;

static i2c_master_dev_handle_t dispositivo_axp2101 = nullptr;
static esp_codec_dev_handle_t altavoz = nullptr;

// Mutex corto: solo protege la toma/copia del framebuffer.
// La inferencia YOLO ocurre DESPUES de liberar la camara.
static SemaphoreHandle_t mutex_camara = nullptr;

// Indicador de FPS real del video mostrado.
static lv_obj_t *etiqueta_fps = nullptr;
static volatile uint32_t contador_frames_video = 0;

// Buffers reutilizables para la tarea de deteccion.
// Evitamos malloc/free en cada inferencia.
static uint8_t *buffer_deteccion_rgb565 = nullptr;
static uint8_t *buffer_deteccion_rgb888 = nullptr;

// ============================================================
// VISTA
// ============================================================

struct VistaCamara {
    uint8_t *buffer_rgb565 = nullptr;
    lv_obj_t *lienzo = nullptr;
    lv_obj_t *capa_detecciones = nullptr;
};

// ============================================================
// FIRMA VISUAL / HISTORIAL / RASTREO
// ============================================================

struct FirmaVisual {
    uint64_t hash = 0;
    uint8_t promedio_r = 0;
    uint8_t promedio_g = 0;
    uint8_t promedio_b = 0;
    uint16_t aspecto_x1000 = 0;
};

struct FirmaRegistrada {
    int id = 0;
    FirmaVisual firma;
};

struct RastreoMartillo {
    bool activo = false;
    int historial_id = 0;

    int x1 = 0;
    int y1 = 0;
    int x2 = 0;
    int y2 = 0;

    FirmaVisual firma;
    uint64_t ultima_vez_ms = 0;
};

struct DeteccionNueva {
    int id = 0;
    int categoria = -1;

    int x1 = 0;
    int y1 = 0;
    int x2 = 0;
    int y2 = 0;

    float confianza = 0.0f;
    FirmaVisual firma;
};

static FirmaRegistrada firmas_historial[MAX_HISTORIAL];
static int cantidad_firmas_historial = 0;
static int ultimo_id_historial = 0;

static RastreoMartillo rastreos[MAX_RASTREOS];


// ============================================================
// DIAGNOSTICO DE MEMORIA
// ============================================================

static void registrar_memoria(const char *etapa)
{
    const size_t ram_libre =
        heap_caps_get_free_size(
            MALLOC_CAP_INTERNAL |
            MALLOC_CAP_8BIT
        );

    const size_t ram_bloque =
        heap_caps_get_largest_free_block(
            MALLOC_CAP_INTERNAL |
            MALLOC_CAP_8BIT
        );

    const size_t psram_libre =
        heap_caps_get_free_size(
            MALLOC_CAP_SPIRAM |
            MALLOC_CAP_8BIT
        );

    const size_t psram_bloque =
        heap_caps_get_largest_free_block(
            MALLOC_CAP_SPIRAM |
            MALLOC_CAP_8BIT
        );

    ESP_LOGI(
        TAG,
        "MEMORIA [%s] RAM_LIBRE=%u RAM_BLOQUE=%u PSRAM_LIBRE=%u PSRAM_BLOQUE=%u",
        etapa,
        static_cast<unsigned>(ram_libre),
        static_cast<unsigned>(ram_bloque),
        static_cast<unsigned>(psram_libre),
        static_cast<unsigned>(psram_bloque)
    );
}


// ============================================================
// UTILIDADES
// ============================================================

static int limitar(int valor, int minimo, int maximo)
{
    if (valor < minimo) return minimo;
    if (valor > maximo) return maximo;
    return valor;
}

static int abs_int(int valor)
{
    return valor < 0 ? -valor : valor;
}

static uint64_t ahora_ms(void)
{
    return static_cast<uint64_t>(
        esp_timer_get_time() / 1000LL
    );
}

static void actualizar_estado(
    lv_obj_t *etiqueta,
    const char *texto,
    uint32_t color
)
{
    if (etiqueta == nullptr) return;

    if (bsp_display_lock(0)) {
        lv_label_set_text(etiqueta, texto);

        lv_obj_set_style_text_color(
            etiqueta,
            lv_color_hex(color),
            LV_PART_MAIN
        );

        lv_obj_move_foreground(etiqueta);
        bsp_display_unlock();
    }
}

static void mostrar_estado_video(
    lv_obj_t *etiqueta
)
{
    char texto[64];

    snprintf(
        texto,
        sizeof(texto),
        "VIDEO AUTO | REG: %d",
        ultimo_id_historial
    );

    actualizar_estado(
        etiqueta,
        texto,
        0xFFFFFF
    );
}

// ============================================================
// BOTON POWER
// ============================================================

static esp_err_t axp2101_leer_registro(
    uint8_t registro,
    uint8_t *valor
)
{
    if (
        dispositivo_axp2101 == nullptr ||
        valor == nullptr
    ) {
        return ESP_ERR_INVALID_STATE;
    }

    return i2c_master_transmit_receive(
        dispositivo_axp2101,
        &registro,
        1,
        valor,
        1,
        1000
    );
}

static esp_err_t axp2101_escribir_registro(
    uint8_t registro,
    uint8_t valor
)
{
    if (dispositivo_axp2101 == nullptr) {
        return ESP_ERR_INVALID_STATE;
    }

    const uint8_t datos[2] = {
        registro,
        valor
    };

    return i2c_master_transmit(
        dispositivo_axp2101,
        datos,
        sizeof(datos),
        1000
    );
}

static esp_err_t iniciar_boton_power(void)
{
    i2c_master_bus_handle_t bus_i2c = nullptr;

    esp_err_t error = i2c_master_get_bus_handle(
        BSP_I2C_NUM,
        &bus_i2c
    );

    if (
        error != ESP_OK ||
        bus_i2c == nullptr
    ) {
        ESP_LOGE(
            TAG,
            "No se obtuvo bus I2C: %s",
            esp_err_to_name(error)
        );

        return error != ESP_OK
            ? error
            : ESP_FAIL;
    }

    i2c_device_config_t configuracion = {};
    configuracion.dev_addr_length = I2C_ADDR_BIT_LEN_7;
    configuracion.device_address = AXP2101_DIRECCION;
    configuracion.scl_speed_hz = 400000;

    error = i2c_master_bus_add_device(
        bus_i2c,
        &configuracion,
        &dispositivo_axp2101
    );

    if (error != ESP_OK) {
        ESP_LOGE(
            TAG,
            "No se pudo registrar AXP2101: %s",
            esp_err_to_name(error)
        );
        return error;
    }

    uint8_t habilitados = 0;

    error = axp2101_leer_registro(
        AXP2101_IRQ_HABILITAR_1,
        &habilitados
    );

    if (error != ESP_OK) return error;

    error = axp2101_escribir_registro(
        AXP2101_IRQ_HABILITAR_1,
        static_cast<uint8_t>(
            habilitados |
            AXP2101_MASCARA_POWER
        )
    );

    if (error != ESP_OK) return error;

    error = axp2101_escribir_registro(
        AXP2101_IRQ_ESTADO_1,
        AXP2101_MASCARA_POWER
    );

    if (error == ESP_OK) {
        ESP_LOGI(TAG, "Boton POWER preparado");
    }

    return error;
}

static uint8_t leer_evento_power(void)
{
    uint8_t estado = 0;

    if (
        axp2101_leer_registro(
            AXP2101_IRQ_ESTADO_1,
            &estado
        ) != ESP_OK
    ) {
        return 0;
    }

    estado &= AXP2101_MASCARA_POWER;

    if (estado != 0) {
        axp2101_escribir_registro(
            AXP2101_IRQ_ESTADO_1,
            estado
        );
    }

    return static_cast<uint8_t>(
        estado >> 2
    );
}

// ============================================================
// VIDEO RGB565
// ============================================================

static bool iniciar_vista_video(
    lv_obj_t *pantalla,
    lv_obj_t *etiqueta_estado,
    VistaCamara &vista
)
{
    const size_t bytes_video =
        static_cast<size_t>(FOTO_ANCHO) *
        static_cast<size_t>(FOTO_ALTO) *
        2U;

    vista.buffer_rgb565 = static_cast<uint8_t *>(
        heap_caps_malloc(
            bytes_video,
            MALLOC_CAP_SPIRAM |
            MALLOC_CAP_8BIT
        )
    );

    if (vista.buffer_rgb565 == nullptr) {
        ESP_LOGE(
            TAG,
            "No hay memoria para buffer de video"
        );
        return false;
    }

    memset(
        vista.buffer_rgb565,
        0,
        bytes_video
    );

    if (!bsp_display_lock(0)) {
        return false;
    }

    vista.lienzo = lv_canvas_create(pantalla);

    // GC0308 entrega RGB565 big-endian.
    lv_canvas_set_buffer(
        vista.lienzo,
        vista.buffer_rgb565,
        FOTO_ANCHO,
        FOTO_ALTO,
        LV_COLOR_FORMAT_RGB565_SWAPPED
    );

    lv_obj_set_pos(
        vista.lienzo,
        0,
        0
    );

    lv_obj_remove_flag(
        vista.lienzo,
        LV_OBJ_FLAG_SCROLLABLE
    );

    lv_obj_move_background(
        vista.lienzo
    );

    lv_obj_move_foreground(
        etiqueta_estado
    );

    bsp_display_unlock();

    return true;
}

static void borrar_detecciones(
    VistaCamara &vista
)
{
    if (
        vista.capa_detecciones == nullptr
    ) {
        return;
    }

    if (bsp_display_lock(0)) {
        lv_obj_delete(
            vista.capa_detecciones
        );

        vista.capa_detecciones = nullptr;

        bsp_display_unlock();
    }
}

static bool actualizar_video_con_frame(
    VistaCamara &vista,
    const camera_fb_t *frame
)
{
    if (
        frame == nullptr ||
        vista.buffer_rgb565 == nullptr ||
        vista.lienzo == nullptr
    ) {
        return false;
    }

    if (
        frame->width != FOTO_ANCHO ||
        frame->height != FOTO_ALTO ||
        frame->format != PIXFORMAT_RGB565
    ) {
        ESP_LOGE(
            TAG,
            "Frame inesperado: %ux%u formato=%d",
            static_cast<unsigned>(
                frame->width
            ),
            static_cast<unsigned>(
                frame->height
            ),
            static_cast<int>(
                frame->format
            )
        );

        return false;
    }

    const size_t bytes_esperados =
        static_cast<size_t>(FOTO_ANCHO) *
        static_cast<size_t>(FOTO_ALTO) *
        2U;

    if (
        frame->len < bytes_esperados
    ) {
        ESP_LOGE(
            TAG,
            "Frame demasiado pequeno: %u bytes",
            static_cast<unsigned>(
                frame->len
            )
        );

        return false;
    }

    if (bsp_display_lock(0)) {
        memcpy(
            vista.buffer_rgb565,
            frame->buf,
            bytes_esperados
        );

        lv_obj_invalidate(
            vista.lienzo
        );

        bsp_display_unlock();

        contador_frames_video++;

        return true;
    }

    return false;
}

static void mostrar_video_un_frame(
    VistaCamara &vista
)
{
    if (mutex_camara == nullptr) {
        return;
    }

    if (
        xSemaphoreTake(
            mutex_camara,
            pdMS_TO_TICKS(10)
        ) != pdTRUE
    ) {
        // Si YOLO esta copiando un frame, no esperamos:
        // dejamos que LVGL mantenga el ultimo cuadro.
        return;
    }

    camera_fb_t *frame =
        esp_camera_fb_get();

    if (frame == nullptr) {
        xSemaphoreGive(
            mutex_camara
        );

        ESP_LOGW(
            TAG,
            "No se obtuvo frame de video"
        );

        return;
    }

    actualizar_video_con_frame(
        vista,
        frame
    );

    esp_camera_fb_return(
        frame
    );

    xSemaphoreGive(
        mutex_camara
    );
}

// ============================================================
// CAJAS YOLO
// ============================================================

template <typename Resultado>
static void obtener_caja(
    const Resultado &resultado,
    int &x1,
    int &y1,
    int &x2,
    int &y2
)
{
    x1 = static_cast<int>(
        resultado.box[0]
    );

    y1 = static_cast<int>(
        resultado.box[1]
    );

    x2 = static_cast<int>(
        resultado.box[2]
    );

    y2 = static_cast<int>(
        resultado.box[3]
    );

    if (
        x2 > FOTO_ANCHO ||
        y2 > FOTO_ALTO
    ) {
        x1 =
            x1 * FOTO_ANCHO /
            MODELO_ANCHO;

        y1 =
            y1 * FOTO_ALTO /
            MODELO_ALTO;

        x2 =
            x2 * FOTO_ANCHO /
            MODELO_ANCHO;

        y2 =
            y2 * FOTO_ALTO /
            MODELO_ALTO;
    }

    x1 = limitar(
        x1,
        0,
        FOTO_ANCHO - 1
    );

    y1 = limitar(
        y1,
        0,
        FOTO_ALTO - 1
    );

    x2 = limitar(
        x2,
        0,
        FOTO_ANCHO - 1
    );

    y2 = limitar(
        y2,
        0,
        FOTO_ALTO - 1
    );
}

static lv_obj_t *crear_capa_detecciones(
    lv_obj_t *pantalla
)
{
    lv_obj_t *capa =
        lv_obj_create(
            pantalla
        );

    lv_obj_set_pos(
        capa,
        0,
        0
    );

    lv_obj_set_size(
        capa,
        FOTO_ANCHO,
        FOTO_ALTO
    );

    lv_obj_set_style_bg_opa(
        capa,
        LV_OPA_TRANSP,
        LV_PART_MAIN
    );

    lv_obj_set_style_border_width(
        capa,
        0,
        LV_PART_MAIN
    );

    lv_obj_set_style_pad_all(
        capa,
        0,
        LV_PART_MAIN
    );

    lv_obj_remove_flag(
        capa,
        LV_OBJ_FLAG_SCROLLABLE
    );

    return capa;
}

static void dibujar_deteccion_nueva(
    lv_obj_t *capa,
    const DeteccionNueva &deteccion
)
{
    const int ancho =
        deteccion.x2 -
        deteccion.x1;

    const int alto =
        deteccion.y2 -
        deteccion.y1;

    if (
        ancho < 3 ||
        alto < 3
    ) {
        return;
    }

    uint32_t color_hex =
        0xFFFFFF;

    const char *nombre_clase =
        "OBJETO";

    // Clase 0: hammer.
    if (deteccion.categoria == 0) {
        color_hex = 0xFF3030;
        nombre_clase = "MARTILLO";
    }
    // Clase 1: plier.
    else if (deteccion.categoria == 1) {
        color_hex = 0x00D9FF;
        nombre_clase = "PINZA";
    }

    lv_obj_t *caja =
        lv_obj_create(capa);

    lv_obj_set_pos(
        caja,
        deteccion.x1,
        deteccion.y1
    );

    lv_obj_set_size(
        caja,
        ancho,
        alto
    );

    lv_obj_set_style_bg_opa(
        caja,
        LV_OPA_TRANSP,
        LV_PART_MAIN
    );

    lv_obj_set_style_border_color(
        caja,
        lv_color_hex(color_hex),
        LV_PART_MAIN
    );

    lv_obj_set_style_border_width(
        caja,
        3,
        LV_PART_MAIN
    );

    lv_obj_set_style_radius(
        caja,
        0,
        LV_PART_MAIN
    );

    lv_obj_set_style_pad_all(
        caja,
        0,
        LV_PART_MAIN
    );

    lv_obj_remove_flag(
        caja,
        LV_OBJ_FLAG_SCROLLABLE
    );

    char texto[64];

    snprintf(
        texto,
        sizeof(texto),
        "%s %.0f%%",
        nombre_clase,
        deteccion.confianza * 100.0f
    );

    lv_obj_t *etiqueta =
        lv_label_create(capa);

    lv_label_set_text(
        etiqueta,
        texto
    );

    lv_obj_set_style_text_color(
        etiqueta,
        lv_color_hex(color_hex),
        LV_PART_MAIN
    );

    lv_obj_set_style_bg_color(
        etiqueta,
        lv_color_hex(0x000000),
        LV_PART_MAIN
    );

    lv_obj_set_style_bg_opa(
        etiqueta,
        LV_OPA_80,
        LV_PART_MAIN
    );

    lv_obj_set_style_pad_all(
        etiqueta,
        2,
        LV_PART_MAIN
    );

    lv_obj_set_pos(
        etiqueta,
        deteccion.x1,
        deteccion.y1 > 18
            ? deteccion.y1 - 18
            : deteccion.y1
    );
}

static bool iniciar_audio(void)
{
    esp_err_t error =
        bsp_audio_init(nullptr);

    if (
        error != ESP_OK &&
        error != ESP_ERR_INVALID_STATE
    ) {
        ESP_LOGE(
            TAG,
            "No se pudo iniciar audio: %s",
            esp_err_to_name(error)
        );

        return false;
    }

    altavoz =
        bsp_audio_codec_speaker_init();

    if (altavoz == nullptr) {
        ESP_LOGE(
            TAG,
            "No se pudo iniciar altavoz"
        );

        return false;
    }

    ESP_LOGI(
        TAG,
        "Altavoz preparado"
    );

    return true;
}

static void reproducir_voz_martillo(void)
{
    if (altavoz == nullptr) {
        return;
    }

    esp_codec_dev_sample_info_t formato = {};

    formato.sample_rate = 22050;
    formato.channel = 1;
    formato.bits_per_sample = 16;

    if (
        esp_codec_dev_open(
            altavoz,
            &formato
        ) != ESP_CODEC_DEV_OK
    ) {
        ESP_LOGE(
            TAG,
            "No se pudo abrir altavoz"
        );

        return;
    }

    esp_codec_dev_set_out_vol(
        altavoz,
        75
    );

    const int resultado =
        esp_codec_dev_write(
            altavoz,
            const_cast<uint8_t *>(
                voz_martillo_pcm
            ),
            static_cast<int>(
                voz_martillo_pcm_len
            )
        );

    if (
        resultado !=
        ESP_CODEC_DEV_OK
    ) {
        ESP_LOGE(
            TAG,
            "Error de audio: %d",
            resultado
        );
    }

    esp_codec_dev_close(
        altavoz
    );
}

// ============================================================
// RGB565 -> FIRMA VISUAL
// ============================================================

static void leer_pixel_rgb565_be(
    const uint8_t *buffer,
    int x,
    int y,
    int &r,
    int &g,
    int &b
)
{
    x = limitar(
        x,
        0,
        FOTO_ANCHO - 1
    );

    y = limitar(
        y,
        0,
        FOTO_ALTO - 1
    );

    const size_t indice =
        (
            static_cast<size_t>(y) *
            FOTO_ANCHO +
            static_cast<size_t>(x)
        ) * 2U;

    const uint16_t pixel =
        static_cast<uint16_t>(
            (
                static_cast<uint16_t>(
                    buffer[indice]
                ) << 8
            ) |
            buffer[indice + 1]
        );

    const int r5 =
        (pixel >> 11) & 0x1F;

    const int g6 =
        (pixel >> 5) & 0x3F;

    const int b5 =
        pixel & 0x1F;

    r = r5 * 255 / 31;
    g = g6 * 255 / 63;
    b = b5 * 255 / 31;
}

static FirmaVisual calcular_firma_visual(
    const uint8_t *buffer,
    int x1,
    int y1,
    int x2,
    int y2
)
{
    FirmaVisual firma;

    if (
        buffer == nullptr ||
        x2 <= x1 ||
        y2 <= y1
    ) {
        return firma;
    }

    const int ancho =
        x2 - x1 + 1;

    const int alto =
        y2 - y1 + 1;

    // Recortamos un 10% del borde para reducir influencia del fondo.
    int ix1 =
        x1 + ancho / 10;

    int iy1 =
        y1 + alto / 10;

    int ix2 =
        x2 - ancho / 10;

    int iy2 =
        y2 - alto / 10;

    if (ix2 <= ix1) {
        ix1 = x1;
        ix2 = x2;
    }

    if (iy2 <= iy1) {
        iy1 = y1;
        iy2 = y2;
    }

    uint8_t luminancias[64];

    int suma_luminancia = 0;
    int suma_r = 0;
    int suma_g = 0;
    int suma_b = 0;

    int indice_muestra = 0;

    for (int gy = 0; gy < 8; ++gy) {
        for (int gx = 0; gx < 8; ++gx) {
            const int x =
                ix1 +
                (
                    (2 * gx + 1) *
                    (ix2 - ix1)
                ) / 16;

            const int y =
                iy1 +
                (
                    (2 * gy + 1) *
                    (iy2 - iy1)
                ) / 16;

            int r = 0;
            int g = 0;
            int b = 0;

            leer_pixel_rgb565_be(
                buffer,
                x,
                y,
                r,
                g,
                b
            );

            // Luma aproximada.
            const int lum =
                (
                    77 * r +
                    150 * g +
                    29 * b
                ) >> 8;

            luminancias[
                indice_muestra++
            ] =
                static_cast<uint8_t>(
                    limitar(
                        lum,
                        0,
                        255
                    )
                );

            suma_luminancia += lum;
            suma_r += r;
            suma_g += g;
            suma_b += b;
        }
    }

    const int promedio_lum =
        suma_luminancia / 64;

    uint64_t hash = 0;

    for (int i = 0; i < 64; ++i) {
        if (
            luminancias[i] >=
            promedio_lum
        ) {
            hash |=
                (
                    static_cast<uint64_t>(1)
                    << i
                );
        }
    }

    firma.hash = hash;

    firma.promedio_r =
        static_cast<uint8_t>(
            suma_r / 64
        );

    firma.promedio_g =
        static_cast<uint8_t>(
            suma_g / 64
        );

    firma.promedio_b =
        static_cast<uint8_t>(
            suma_b / 64
        );

    firma.aspecto_x1000 =
        static_cast<uint16_t>(
            limitar(
                alto > 0
                    ? (
                        ancho * 1000 /
                        alto
                    )
                    : 0,
                0,
                65535
            )
        );

    return firma;
}

static int distancia_hamming(
    uint64_t a,
    uint64_t b
)
{
    return __builtin_popcountll(
        a ^ b
    );
}

static bool firmas_similares(
    const FirmaVisual &a,
    const FirmaVisual &b
)
{
    if (
        a.hash == 0 ||
        b.hash == 0
    ) {
        return false;
    }

    const int hamming =
        distancia_hamming(
            a.hash,
            b.hash
        );

    if (
        hamming >
        HAMMING_FIRMA_MAX
    ) {
        return false;
    }

    const int diferencia_color =
        abs_int(
            static_cast<int>(
                a.promedio_r
            ) -
            static_cast<int>(
                b.promedio_r
            )
        ) +
        abs_int(
            static_cast<int>(
                a.promedio_g
            ) -
            static_cast<int>(
                b.promedio_g
            )
        ) +
        abs_int(
            static_cast<int>(
                a.promedio_b
            ) -
            static_cast<int>(
                b.promedio_b
            )
        );

    if (
        diferencia_color >
        DIFERENCIA_COLOR_MAX
    ) {
        return false;
    }

    const int diferencia_aspecto =
        abs_int(
            static_cast<int>(
                a.aspecto_x1000
            ) -
            static_cast<int>(
                b.aspecto_x1000
            )
        );

    if (
        diferencia_aspecto >
        DIFERENCIA_ASPECTO_MAX
    ) {
        return false;
    }

    return true;
}

// ============================================================
// RASTREO
// ============================================================

static float calcular_iou(
    int ax1,
    int ay1,
    int ax2,
    int ay2,
    int bx1,
    int by1,
    int bx2,
    int by2
)
{
    const int ix1 =
        ax1 > bx1 ? ax1 : bx1;

    const int iy1 =
        ay1 > by1 ? ay1 : by1;

    const int ix2 =
        ax2 < bx2 ? ax2 : bx2;

    const int iy2 =
        ay2 < by2 ? ay2 : by2;

    const int iw =
        ix2 - ix1;

    const int ih =
        iy2 - iy1;

    if (
        iw <= 0 ||
        ih <= 0
    ) {
        return 0.0f;
    }

    const int interseccion =
        iw * ih;

    const int area_a =
        (
            ax2 - ax1
        ) *
        (
            ay2 - ay1
        );

    const int area_b =
        (
            bx2 - bx1
        ) *
        (
            by2 - by1
        );

    const int union_area =
        area_a +
        area_b -
        interseccion;

    if (union_area <= 0) {
        return 0.0f;
    }

    return
        static_cast<float>(
            interseccion
        ) /
        static_cast<float>(
            union_area
        );
}

static int distancia_centro_cuadrada(
    int ax1,
    int ay1,
    int ax2,
    int ay2,
    int bx1,
    int by1,
    int bx2,
    int by2
)
{
    const int acx =
        (ax1 + ax2) / 2;

    const int acy =
        (ay1 + ay2) / 2;

    const int bcx =
        (bx1 + bx2) / 2;

    const int bcy =
        (by1 + by2) / 2;

    const int dx =
        acx - bcx;

    const int dy =
        acy - bcy;

    return dx * dx + dy * dy;
}

static void expirar_rastreos(
    uint64_t tiempo_ms
)
{
    for (
        int i = 0;
        i < MAX_RASTREOS;
        ++i
    ) {
        if (!rastreos[i].activo) {
            continue;
        }

        if (
            tiempo_ms -
            rastreos[i].ultima_vez_ms >
            RASTREO_EXPIRA_MS
        ) {
            rastreos[i].activo =
                false;
        }
    }
}

static int buscar_rastreo_activo(
    int x1,
    int y1,
    int x2,
    int y2,
    const FirmaVisual &firma,
    uint64_t tiempo_ms
)
{
    expirar_rastreos(
        tiempo_ms
    );

    int mejor_indice = -1;
    float mejor_iou = 0.0f;

    for (
        int i = 0;
        i < MAX_RASTREOS;
        ++i
    ) {
        if (!rastreos[i].activo) {
            continue;
        }

        const float iou =
            calcular_iou(
                x1,
                y1,
                x2,
                y2,
                rastreos[i].x1,
                rastreos[i].y1,
                rastreos[i].x2,
                rastreos[i].y2
            );

        const int distancia2 =
            distancia_centro_cuadrada(
                x1,
                y1,
                x2,
                y2,
                rastreos[i].x1,
                rastreos[i].y1,
                rastreos[i].x2,
                rastreos[i].y2
            );

        const bool cerca =
            distancia2 <=
            (
                DISTANCIA_CENTRO_MAX *
                DISTANCIA_CENTRO_MAX
            );

        const bool apariencia =
            firmas_similares(
                firma,
                rastreos[i].firma
            );

        if (
            iou >=
            IOU_MINIMA_RASTREO ||
            (
                cerca &&
                apariencia
            )
        ) {
            if (
                mejor_indice < 0 ||
                iou > mejor_iou
            ) {
                mejor_indice = i;
                mejor_iou = iou;
            }
        }
    }

    return mejor_indice;
}

static void actualizar_firma_historial_en_ram(
    int id,
    const FirmaVisual &firma
)
{
    for (
        int i = 0;
        i < cantidad_firmas_historial;
        ++i
    ) {
        if (
            firmas_historial[i].id ==
            id
        ) {
            // Durante la misma sesion actualizamos la apariencia
            // conocida para seguir mejor al objeto cuando se mueve.
            firmas_historial[i].firma =
                firma;

            return;
        }
    }
}

static void actualizar_rastreo(
    int indice,
    int x1,
    int y1,
    int x2,
    int y2,
    const FirmaVisual &firma,
    uint64_t tiempo_ms
)
{
    if (
        indice < 0 ||
        indice >= MAX_RASTREOS
    ) {
        return;
    }

    rastreos[indice].x1 = x1;
    rastreos[indice].y1 = y1;
    rastreos[indice].x2 = x2;
    rastreos[indice].y2 = y2;

    rastreos[indice].firma =
        firma;

    rastreos[indice].ultima_vez_ms =
        tiempo_ms;

    actualizar_firma_historial_en_ram(
        rastreos[indice].historial_id,
        firma
    );
}

static int crear_rastreo(
    int historial_id,
    int x1,
    int y1,
    int x2,
    int y2,
    const FirmaVisual &firma,
    uint64_t tiempo_ms
)
{
    int indice = -1;

    for (
        int i = 0;
        i < MAX_RASTREOS;
        ++i
    ) {
        if (!rastreos[i].activo) {
            indice = i;
            break;
        }
    }

    // Si todos estan ocupados, reemplazamos el mas antiguo.
    if (indice < 0) {
        uint64_t mas_antiguo =
            UINT64_MAX;

        for (
            int i = 0;
            i < MAX_RASTREOS;
            ++i
        ) {
            if (
                rastreos[i].ultima_vez_ms <
                mas_antiguo
            ) {
                mas_antiguo =
                    rastreos[i].ultima_vez_ms;

                indice = i;
            }
        }
    }

    if (indice < 0) {
        return -1;
    }

    rastreos[indice].activo = true;

    rastreos[indice].historial_id =
        historial_id;

    rastreos[indice].x1 = x1;
    rastreos[indice].y1 = y1;
    rastreos[indice].x2 = x2;
    rastreos[indice].y2 = y2;

    rastreos[indice].firma =
        firma;

    rastreos[indice].ultima_vez_ms =
        tiempo_ms;

    return indice;
}

// ============================================================
// HISTORIAL PERSISTENTE
// ============================================================

static bool iniciar_storage(void)
{
    esp_vfs_spiffs_conf_t configuracion = {};

    configuracion.base_path =
        BASE_STORAGE;

    configuracion.partition_label =
        ETIQUETA_STORAGE;

    configuracion.max_files = 8;

    configuracion.format_if_mount_failed =
        true;

    esp_err_t error =
        esp_vfs_spiffs_register(
            &configuracion
        );

    if (error != ESP_OK) {
        ESP_LOGE(
            TAG,
            "No se monto SPIFFS storage: %s",
            esp_err_to_name(error)
        );

        return false;
    }

    size_t total = 0;
    size_t usado = 0;

    error = esp_spiffs_info(
        ETIQUETA_STORAGE,
        &total,
        &usado
    );

    if (error == ESP_OK) {
        ESP_LOGI(
            TAG,
            "Storage: %u KB usados de %u KB",
            static_cast<unsigned>(
                usado / 1024
            ),
            static_cast<unsigned>(
                total / 1024
            )
        );
    }

    return true;
}

static bool asegurar_archivo_historial(void)
{
    struct stat info;

    if (
        stat(
            ARCHIVO_HISTORIAL,
            &info
        ) == 0
    ) {
        return true;
    }

    FILE *archivo =
        fopen(
            ARCHIVO_HISTORIAL,
            "w"
        );

    if (archivo == nullptr) {
        ESP_LOGE(
            TAG,
            "No se pudo crear historial.csv"
        );

        return false;
    }

    fprintf(
        archivo,
        "id,ms,conf,x1,y1,x2,y2,hash,r,g,b,aspecto,archivo\n"
    );

    fclose(
        archivo
    );

    return true;
}


// ============================================================
// RESET DE DATOS UNA SOLA VEZ
// ============================================================

static void resetear_datos_iniciales_una_vez(void)
{
    static constexpr const char *MARCADOR_RESET =
        "/storage/reset_datos_20260819.ok";

    struct stat info = {};

    // Si existe, ya hicimos el reset anteriormente.
    if (
        stat(
            MARCADOR_RESET,
            &info
        ) == 0
    ) {
        return;
    }

    ESP_LOGW(
        TAG,
        "RESET INICIAL: limpiando historial, Excel y capturas"
    );

    remove(
        "/storage/historial.csv"
    );

    remove(
        "/storage/detecciones_excel.csv"
    );

    remove(
        "/storage/ultima_deteccion.jpg"
    );

    DIR *directorio =
        opendir(
            "/storage"
        );

    int eliminadas = 0;

    if (directorio != nullptr) {
        while (
            auto *entrada =
                readdir(
                    directorio
                )
        ) {
            if (
                strstr(
                    entrada->d_name,
                    "captura_"
                ) != entrada->d_name
            ) {
                continue;
            }

            if (
                strstr(
                    entrada->d_name,
                    ".jpg"
                ) == nullptr
            ) {
                continue;
            }

            char ruta[320];

            snprintf(
                ruta,
                sizeof(ruta),
                "/storage/%s",
                entrada->d_name
            );

            if (
                remove(
                    ruta
                ) == 0
            ) {
                eliminadas++;
            }
        }

        closedir(
            directorio
        );
    }

    cantidad_firmas_historial = 0;
    ultimo_id_historial = 0;

    memset(
        firmas_historial,
        0,
        sizeof(firmas_historial)
    );

    memset(
        rastreos,
        0,
        sizeof(rastreos)
    );

    // Creamos el marcador para que este borrado NO ocurra otra vez.
    FILE *marca =
        fopen(
            MARCADOR_RESET,
            "w"
        );

    if (marca != nullptr) {
        fprintf(
            marca,
            "RESET_OK\n"
        );

        fclose(
            marca
        );
    }

    ESP_LOGW(
        TAG,
        "RESET INICIAL TERMINADO: historial=0 capturas_eliminadas=%d",
        eliminadas
    );
}

static bool cargar_historial(void)
{
    cantidad_firmas_historial = 0;
    ultimo_id_historial = 0;

    if (!asegurar_archivo_historial()) {
        return false;
    }

    FILE *archivo =
        fopen(
            ARCHIVO_HISTORIAL,
            "r"
        );

    if (archivo == nullptr) {
        ESP_LOGE(
            TAG,
            "No se pudo abrir historial.csv"
        );

        return false;
    }

    char linea[256];

    // Saltar cabecera.
    fgets(
        linea,
        sizeof(linea),
        archivo
    );

    while (
        fgets(
            linea,
            sizeof(linea),
            archivo
        ) != nullptr
    ) {
        int id = 0;
        unsigned long long tiempo = 0;
        float confianza = 0.0f;

        int x1 = 0;
        int y1 = 0;
        int x2 = 0;
        int y2 = 0;

        unsigned long long hash = 0;

        int r = 0;
        int g = 0;
        int b = 0;
        int aspecto = 0;

        char nombre_archivo[96] = {};

        const int leidos =
            sscanf(
                linea,
                "%d,%llu,%f,%d,%d,%d,%d,%llx,%d,%d,%d,%d,%95[^\n]",
                &id,
                &tiempo,
                &confianza,
                &x1,
                &y1,
                &x2,
                &y2,
                &hash,
                &r,
                &g,
                &b,
                &aspecto,
                nombre_archivo
            );

        if (leidos < 12) {
            continue;
        }

        if (
            id >
            ultimo_id_historial
        ) {
            ultimo_id_historial = id;
        }

        if (
            cantidad_firmas_historial >=
            MAX_HISTORIAL
        ) {
            continue;
        }

        FirmaRegistrada &registro =
            firmas_historial[
                cantidad_firmas_historial++
            ];

        registro.id = id;

        registro.firma.hash =
            static_cast<uint64_t>(
                hash
            );

        registro.firma.promedio_r =
            static_cast<uint8_t>(
                limitar(
                    r,
                    0,
                    255
                )
            );

        registro.firma.promedio_g =
            static_cast<uint8_t>(
                limitar(
                    g,
                    0,
                    255
                )
            );

        registro.firma.promedio_b =
            static_cast<uint8_t>(
                limitar(
                    b,
                    0,
                    255
                )
            );

        registro.firma.aspecto_x1000 =
            static_cast<uint16_t>(
                limitar(
                    aspecto,
                    0,
                    65535
                )
            );
    }

    fclose(
        archivo
    );

    ESP_LOGI(
        TAG,
        "Historial cargado: %d firmas, ultimo ID=%d",
        cantidad_firmas_historial,
        ultimo_id_historial
    );

    return true;
}

static int buscar_firma_en_historial(
    const FirmaVisual &firma
)
{
    int mejor_indice = -1;
    int mejor_hamming = 1000;

    for (
        int i = 0;
        i < cantidad_firmas_historial;
        ++i
    ) {
        if (
            !firmas_similares(
                firma,
                firmas_historial[i].firma
            )
        ) {
            continue;
        }

        const int hamming =
            distancia_hamming(
                firma.hash,
                firmas_historial[i].firma.hash
            );

        if (
            hamming <
            mejor_hamming
        ) {
            mejor_hamming = hamming;
            mejor_indice = i;
        }
    }

    return mejor_indice;
}


// ============================================================
// DIBUJO DE CAJAS DIRECTAMENTE EN RGB565 PARA JPEG/WEB
// ============================================================

static void poner_pixel_rgb565_be(
    uint8_t *buffer,
    int x,
    int y,
    uint16_t color
)
{
    if (
        buffer == nullptr ||
        x < 0 ||
        y < 0 ||
        x >= FOTO_ANCHO ||
        y >= FOTO_ALTO
    ) {
        return;
    }

    const size_t indice =
        (
            static_cast<size_t>(y) *
            static_cast<size_t>(FOTO_ANCHO) +
            static_cast<size_t>(x)
        ) * 2U;

    // La GC0308 trabaja con RGB565 big-endian.
    buffer[indice] =
        static_cast<uint8_t>(
            (color >> 8) & 0xFF
        );

    buffer[indice + 1] =
        static_cast<uint8_t>(
            color & 0xFF
        );
}

static void dibujar_rectangulo_rgb565(
    uint8_t *buffer,
    int x1,
    int y1,
    int x2,
    int y2,
    uint16_t color,
    int grosor
)
{
    if (buffer == nullptr) return;

    if (x1 < 0) x1 = 0;
    if (y1 < 0) y1 = 0;

    if (x2 >= FOTO_ANCHO) {
        x2 = FOTO_ANCHO - 1;
    }

    if (y2 >= FOTO_ALTO) {
        y2 = FOTO_ALTO - 1;
    }

    if (
        x2 <= x1 ||
        y2 <= y1
    ) {
        return;
    }

    for (int g = 0; g < grosor; ++g) {
        const int izquierda =
            x1 + g;

        const int derecha =
            x2 - g;

        const int arriba =
            y1 + g;

        const int abajo =
            y2 - g;

        if (
            izquierda >= derecha ||
            arriba >= abajo
        ) {
            break;
        }

        for (
            int x = izquierda;
            x <= derecha;
            ++x
        ) {
            poner_pixel_rgb565_be(
                buffer,
                x,
                arriba,
                color
            );

            poner_pixel_rgb565_be(
                buffer,
                x,
                abajo,
                color
            );
        }

        for (
            int y = arriba;
            y <= abajo;
            ++y
        ) {
            poner_pixel_rgb565_be(
                buffer,
                izquierda,
                y,
                color
            );

            poner_pixel_rgb565_be(
                buffer,
                derecha,
                y,
                color
            );
        }
    }
}

static void dibujar_cajas_para_web(
    uint8_t *buffer,
    const DeteccionNueva *detecciones,
    int cantidad
)
{
    if (
        buffer == nullptr ||
        detecciones == nullptr
    ) {
        return;
    }

    for (
        int i = 0;
        i < cantidad;
        ++i
    ) {
        const DeteccionNueva &d =
            detecciones[i];

        // RGB565:
        // MARTILLO = rojo
        // PINZA    = celeste
        const uint16_t color =
            d.categoria == 1
                ? 0x07FF
                : 0xF800;

        // Sombra negra exterior para que la caja destaque.
        dibujar_rectangulo_rgb565(
            buffer,
            d.x1 - 1,
            d.y1 - 1,
            d.x2 + 1,
            d.y2 + 1,
            0x0000,
            1
        );

        // Caja principal.
        dibujar_rectangulo_rgb565(
            buffer,
            d.x1,
            d.y1,
            d.x2,
            d.y2,
            color,
            3
        );
    }
}

// ============================================================
// DIBUJO DE CAJAS DIRECTAMENTE EN RGB565 PARA JPEG/WEB
// ============================================================

static bool guardar_jpeg_actual(
    const uint8_t *buffer_rgb565,
    const char *ruta
)
{
    if (
        buffer_rgb565 == nullptr ||
        ruta == nullptr
    ) {
        return false;
    }

    uint8_t *jpeg = nullptr;
    size_t jpeg_len = 0;

    const size_t bytes_rgb565 =
        static_cast<size_t>(
            FOTO_ANCHO
        ) *
        static_cast<size_t>(
            FOTO_ALTO
        ) *
        2U;

    const bool convertido =
        fmt2jpg(
            const_cast<uint8_t *>(buffer_rgb565),
            bytes_rgb565,
            FOTO_ANCHO,
            FOTO_ALTO,
            PIXFORMAT_RGB565,
            CALIDAD_JPEG,
            &jpeg,
            &jpeg_len
        );

    if (
        !convertido ||
        jpeg == nullptr ||
        jpeg_len == 0
    ) {
        ESP_LOGE(
            TAG,
            "No se pudo convertir captura a JPEG"
        );

        if (jpeg != nullptr) {
            free(jpeg);
        }

        return false;
    }

    FILE *archivo =
        fopen(
            ruta,
            "wb"
        );

    if (archivo == nullptr) {
        ESP_LOGE(
            TAG,
            "No se pudo crear %s",
            ruta
        );

        free(jpeg);
        return false;
    }

    const size_t escritos =
        fwrite(
            jpeg,
            1,
            jpeg_len,
            archivo
        );

    fclose(
        archivo
    );

    free(
        jpeg
    );

    if (
        escritos !=
        jpeg_len
    ) {
        ESP_LOGE(
            TAG,
            "Captura incompleta: %u/%u bytes",
            static_cast<unsigned>(
                escritos
            ),
            static_cast<unsigned>(
                jpeg_len
            )
        );

        remove(
            ruta
        );

        return false;
    }

    ESP_LOGI(
        TAG,
        "Captura guardada: %s (%u bytes)",
        ruta,
        static_cast<unsigned>(
            jpeg_len
        )
    );

    return true;
}

static bool registrar_evento_nuevo(
    const uint8_t *buffer_rgb565,
    DeteccionNueva *nuevos,
    int cantidad_nuevos,
    uint64_t tiempo_ms
)
{
    if (
        nuevos == nullptr ||
        cantidad_nuevos <= 0
    ) {
        return false;
    }

    if (
        cantidad_firmas_historial +
        cantidad_nuevos >
        MAX_HISTORIAL
    ) {
        ESP_LOGE(
            TAG,
            "Historial lleno (%d)",
            MAX_HISTORIAL
        );

        return false;
    }

    char ruta_jpeg[96];
    char nombre_jpeg[64];

    snprintf(
        nombre_jpeg,
        sizeof(nombre_jpeg),
        "captura_%010" PRIu64 ".jpg",
        tiempo_ms
    );

    snprintf(
        ruta_jpeg,
        sizeof(ruta_jpeg),
        "%s/%s",
        BASE_STORAGE,
        nombre_jpeg
    );

    // Una sola fotografia por evento, incluso si aparecen
    // varios martillos nuevos simultaneamente.
    if (
        !guardar_jpeg_actual(
            buffer_rgb565,
            ruta_jpeg
        )
    ) {
        return false;
    }

    FILE *archivo =
        fopen(
            ARCHIVO_HISTORIAL,
            "a"
        );

    if (archivo == nullptr) {
        ESP_LOGE(
            TAG,
            "No se pudo abrir historial para escribir"
        );

        remove(
            ruta_jpeg
        );

        return false;
    }

    const int id_inicial =
        ultimo_id_historial;

    int agregados = 0;

    for (
        int i = 0;
        i < cantidad_nuevos;
        ++i
    ) {
        const int nuevo_id =
            ++ultimo_id_historial;

        nuevos[i].id =
            nuevo_id;

        fprintf(
            archivo,
            "%d,%" PRIu64 ",%.5f,%d,%d,%d,%d,%016" PRIx64 ",%u,%u,%u,%u,%s\n",
            nuevo_id,
            tiempo_ms,
            nuevos[i].confianza,
            nuevos[i].x1,
            nuevos[i].y1,
            nuevos[i].x2,
            nuevos[i].y2,
            nuevos[i].firma.hash,
            static_cast<unsigned>(
                nuevos[i].firma.promedio_r
            ),
            static_cast<unsigned>(
                nuevos[i].firma.promedio_g
            ),
            static_cast<unsigned>(
                nuevos[i].firma.promedio_b
            ),
            static_cast<unsigned>(
                nuevos[i].firma.aspecto_x1000
            ),
            nombre_jpeg
        );

        FirmaRegistrada &registro =
            firmas_historial[
                cantidad_firmas_historial++
            ];

        registro.id =
            nuevo_id;

        registro.firma =
            nuevos[i].firma;

        crear_rastreo(
            nuevo_id,
            nuevos[i].x1,
            nuevos[i].y1,
            nuevos[i].x2,
            nuevos[i].y2,
            nuevos[i].firma,
            tiempo_ms
        );

        agregados++;
    }

    fflush(
        archivo
    );

    fclose(
        archivo
    );

    if (
        agregados !=
        cantidad_nuevos
    ) {
        ultimo_id_historial =
            id_inicial;

        return false;
    }

    ESP_LOGI(
        TAG,
        "Evento guardado: %d herramienta(s) nueva(s). Total=%d",
        cantidad_nuevos,
        ultimo_id_historial
    );

    return true;
}

// ============================================================
// FILTRO DE DUPLICADOS EN UN MISMO FRAME
// ============================================================

static bool coincide_con_nuevo_del_mismo_frame(
    const DeteccionNueva *nuevos,
    int cantidad,
    int x1,
    int y1,
    int x2,
    int y2,
    const FirmaVisual &firma
)
{
    for (
        int i = 0;
        i < cantidad;
        ++i
    ) {
        const float iou =
            calcular_iou(
                x1,
                y1,
                x2,
                y2,
                nuevos[i].x1,
                nuevos[i].y1,
                nuevos[i].x2,
                nuevos[i].y2
            );

        if (
            iou >= 0.45f ||
            firmas_similares(
                firma,
                nuevos[i].firma
            )
        ) {
            return true;
        }
    }

    return false;
}

// ============================================================
// DETECCION AUTOMATICA
// ============================================================

static float calcular_iou_cajas(
    int ax1, int ay1, int ax2, int ay2,
    int bx1, int by1, int bx2, int by2
)
{
    const int ix1 = ax1 > bx1 ? ax1 : bx1;
    const int iy1 = ay1 > by1 ? ay1 : by1;
    const int ix2 = ax2 < bx2 ? ax2 : bx2;
    const int iy2 = ay2 < by2 ? ay2 : by2;
    const int iw = ix2 > ix1 ? ix2 - ix1 : 0;
    const int ih = iy2 > iy1 ? iy2 - iy1 : 0;
    const float inter = static_cast<float>(iw * ih);
    const float area_a = static_cast<float>((ax2 - ax1) * (ay2 - ay1));
    const float area_b = static_cast<float>((bx2 - bx1) * (by2 - by1));
    const float union_area = area_a + area_b - inter;
    if (union_area <= 0.0f) return 0.0f;
    return inter / union_area;
}

static bool detectar_automaticamente(
    lv_obj_t *pantalla,
    lv_obj_t *etiqueta_estado,
    VistaCamara &vista,
    dl::Model *modelo,
    YOLO26 *detector
)
{
    const uint64_t inicio_respuesta_ms =
        ahora_ms();

    const uint64_t tiempo_ms =
        inicio_respuesta_ms;

    if (
        buffer_deteccion_rgb565 == nullptr ||
        buffer_deteccion_rgb888 == nullptr ||
        mutex_camara == nullptr
    ) {
        ESP_LOGE(
            TAG,
            "Buffers de deteccion no preparados"
        );

        return false;
    }

    borrar_detecciones(
        vista
    );

    // ========================================================
    // CAPTURA
    // ========================================================

    if (
        xSemaphoreTake(
            mutex_camara,
            pdMS_TO_TICKS(250)
        ) != pdTRUE
    ) {
        ESP_LOGW(
            TAG,
            "Camara ocupada"
        );
        return false;
    }

    camera_fb_t *frame =
        esp_camera_fb_get();

    if (frame == nullptr) {
        xSemaphoreGive(
            mutex_camara
        );

        ESP_LOGW(
            TAG,
            "No se obtuvo frame para deteccion"
        );

        return false;
    }

    ESP_LOGI(
        TAG,
        "FRAME YOLO -> width=%u height=%u len=%u format=%d",
        static_cast<unsigned>(frame->width),
        static_cast<unsigned>(frame->height),
        static_cast<unsigned>(frame->len),
        static_cast<int>(frame->format)
    );

    const size_t bytes_rgb565 =
        static_cast<size_t>(
            FOTO_ANCHO
        ) *
        static_cast<size_t>(
            FOTO_ALTO
        ) *
        2U;

    if (
        frame->format != PIXFORMAT_RGB565 ||
        frame->width != FOTO_ANCHO ||
        frame->height != FOTO_ALTO ||
        frame->len < bytes_rgb565
    ) {
        ESP_LOGE(
            TAG,
            "Frame invalido: %dx%d len=%u",
            frame->width,
            frame->height,
            static_cast<unsigned>(frame->len)
        );

        esp_camera_fb_return(
            frame
        );

        xSemaphoreGive(
            mutex_camara
        );

        return false;
    }

    memcpy(
        buffer_deteccion_rgb565,
        frame->buf,
        bytes_rgb565
    );

    // DEBUG: guardar exactamente el frame utilizado por YOLO
    if (
        guardar_jpeg_actual(
            buffer_deteccion_rgb565,
            "/storage/debug_entrada_yolo.jpg"
        )
    ) {
        ESP_LOGI(
            TAG,
            "DEBUG YOLO: frame de entrada guardado"
        );
    }
    else {
        ESP_LOGW(
            TAG,
            "DEBUG YOLO: no se pudo guardar frame"
        );
    }

    esp_camera_fb_return(
        frame
    );

    xSemaphoreGive(
        mutex_camara
    );

    // ========================================================
    // CONVERSION
    // ========================================================

    if (
        !fmt2rgb888(
            buffer_deteccion_rgb565,
            bytes_rgb565,
            PIXFORMAT_RGB565,
            buffer_deteccion_rgb888
        )
    ) {
        ESP_LOGE(
            TAG,
            "Error RGB565 -> RGB888"
        );

        return false;
    }

    dl::image::img_t imagen_rgb;

    imagen_rgb.data =
        buffer_deteccion_rgb888;

    imagen_rgb.width =
        FOTO_ANCHO;

    imagen_rgb.height =
        FOTO_ALTO;

    imagen_rgb.pix_type =
        dl::image::DL_IMAGE_PIX_TYPE_RGB888;

    // ========================================================
    // INFERENCIA YOLO
    // ========================================================

    detector->preprocess(
        imagen_rgb
    );

    modelo->run();

    auto resultados =
        detector->postprocess(
            modelo->get_outputs()
        );

    // ========================================================
    // DIAGNOSTICO RAW YOLO
    // Muestra exactamente lo que entrega el modelo ANTES
    // de rescates, filtros, NMS y correcciones de clase.
    // ========================================================
    ESP_LOGI(
        TAG,
        "========== YOLO RAW: %d resultado(s) ==========",
        static_cast<int>(resultados.size())
    );

    for (const auto &raw : resultados) {
        const int raw_categoria =
            static_cast<int>(raw.category);

        int rx1 = 0;
        int ry1 = 0;
        int rx2 = 0;
        int ry2 = 0;

        obtener_caja(
            raw,
            rx1,
            ry1,
            rx2,
            ry2
        );

        const char *raw_nombre =
            (
                raw_categoria >= 0 &&
                raw_categoria < NUM_CLASES_HERRAMIENTAS
            )
                ? CLASES_HERRAMIENTAS[raw_categoria]
                : "DESCONOCIDA";

        ESP_LOGI(
            TAG,
            "YOLO RAW -> clase=%d %s score=%.3f caja=[%d,%d,%d,%d]",
            raw_categoria,
            raw_nombre,
            raw.score,
            rx1,
            ry1,
            rx2,
            ry2
        );
    }

    ESP_LOGI(
        TAG,
        "==============================================="
    );

    static DeteccionNueva visibles[MAX_NUEVOS_POR_FRAME];

    int cantidad_visibles = 0;

    static DeteccionNueva nuevos[MAX_NUEVOS_POR_FRAME];

    int cantidad_nuevos = 0;

    int martillos_detectados = 0;
    int pinzas_detectadas = 0;

    int detecciones_validas = 0;
    int cajas_secundarias = 0;
    int duplicados_rastreo = 0;
    int duplicados_historial = 0;

    expirar_rastreos(
        tiempo_ms
    );

    // ========================================================
    // FILTRO DE CAJAS INTERNAS / DUPLICADAS
    // ========================================================
    //
    // Si YOLO marca la pinza completa y tambien el mango,
    // la caja pequena suele quedar dentro de la grande.
    // Se descarta la pequena cuando:
    // - ambas son de la misma clase;
    // - la otra caja tiene al menos 30% mas area;
    // - 65% o mas de la caja pequena esta dentro de la grande.
    // ========================================================

    for (
        const auto &resultado :
        resultados
    ) {
        int categoria =
            static_cast<int>(
                resultado.category
            );

        float confianza =
            resultado.score;

        // Solo validamos la clase aqui.
        // La confianza se filtra DESPUES del rescate.
        if (
            categoria < 0 ||
            categoria >= NUM_CLASES_HERRAMIENTAS
        ) {
            continue;
        }

        int x1 = 0;
        int y1 = 0;
        int x2 = 0;
        int y2 = 0;

        obtener_caja(
            resultado,
            x1,
            y1,
            x2,
            y2
        );

        // ====================================================
        // RESCATE_PLIER_MISMO_OBJETO
        // Solo corrige HAMMER -> PLIER cuando existe una
        // prediccion PLIER sobre la MISMA zona.
        // Objetos separados se conservan independientemente.
        // ====================================================

        if (categoria == 0) {
            float mejor_plier_score = 0.0f;
            float mejor_plier_iou = 0.0f;

            for (
                const auto &candidato :
                resultados
            ) {
                const int cat_candidato =
                    static_cast<int>(
                        candidato.category
                    );

                if (cat_candidato != 1) {
                    continue;
                }

                // Evidencia minima para considerar la clase plier.
                if (candidato.score < 0.10f) {
                    continue;
                }

                int px1 = 0;
                int py1 = 0;
                int px2 = 0;
                int py2 = 0;

                obtener_caja(
                    candidato,
                    px1,
                    py1,
                    px2,
                    py2
                );

                const float iou_plier =
                    calcular_iou_cajas(
                        x1,
                        y1,
                        x2,
                        y2,
                        px1,
                        py1,
                        px2,
                        py2
                    );

                // Si no hay suficiente superposicion,
                // es otra herramienta y NO se mezclan.
                if (iou_plier < 0.30f) {
                    continue;
                }

                if (
                    candidato.score >
                    mejor_plier_score
                ) {
                    mejor_plier_score =
                        candidato.score;

                    mejor_plier_iou =
                        iou_plier;
                }
            }

            if (mejor_plier_score >= CONFIANZA_PLIER) {
                bool rescatar =
                    false;

                // Conflicto normal:
                // permitimos que plier tenga hasta 0.55 menos
                // que hammer si ambas cajas son el mismo objeto.
                if (
                    confianza < 0.96f &&
                    mejor_plier_score + 0.45f >= confianza
                ) {
                    rescatar = true;
                }

                // Hammer extremadamente alto: exigimos
                // evidencia fuerte de plier.
                if (
                    confianza >= 0.96f &&
                    mejor_plier_score >= 0.50f
                ) {
                    rescatar = true;
                }

                if (rescatar) {
                    ESP_LOGI(
                        TAG,
                        "RESCATE PLIER: hammer %.2f -> plier %.2f IoU=%.2f",
                        confianza,
                        mejor_plier_score,
                        mejor_plier_iou
                    );

                    categoria = 1;
                    confianza =
                        mejor_plier_score;
                }
                else {
                    ESP_LOGI(
                        TAG,
                        "CONSERVA HAMMER: hammer %.2f vs plier %.2f IoU=%.2f",
                        confianza,
                        mejor_plier_score,
                        mejor_plier_iou
                    );
                }
            }
        }


        // FILTRO_FINAL_DESPUES_RESCATE
        const float confianza_minima_final =
            categoria == 1
                ? CONFIANZA_PLIER
                : CONFIANZA_HAMMER;

        if (confianza < confianza_minima_final) {
            ESP_LOGI(
                TAG,
                "Descartada por confianza final: %s %.2f < %.2f",
                CLASES_HERRAMIENTAS[categoria],
                confianza,
                confianza_minima_final
            );
            continue;
        }

        const int ancho =
            x2 - x1;

        const int alto =
            y2 - y1;

        if (
            ancho < 5 ||
            alto < 5
        ) {
            continue;
        }

        const int area_actual =
            ancho * alto;

        // FILTRO FINAL DE DUPLICADOS Y CONFLICTOS
        bool duplicada_por_iou = false;
        bool hammer_confundido_con_plier = false;

        for (const auto &otro : resultados) {
            const int otra_categoria = static_cast<int>(otro.category);

            if (otra_categoria < 0 || otra_categoria >= NUM_CLASES_HERRAMIENTAS) {
                continue;
            }

            int ox1 = 0, oy1 = 0, ox2 = 0, oy2 = 0;
            obtener_caja(otro, ox1, oy1, ox2, oy2);

            const float iou = calcular_iou_cajas(
                x1, y1, x2, y2,
                ox1, oy1, ox2, oy2
            );

            // Misma clase: NMS normal.
            if (otra_categoria == categoria) {
                const float otro_umbral =
                    otra_categoria == 1 ? CONFIANZA_PLIER : CONFIANZA_HAMMER;

                if (otro.score < otro_umbral) continue;
                if (otro.score <= confianza) continue;

                if (iou >= 0.55f) {
                    duplicada_por_iou = true;
                    ESP_LOGI(
                        TAG,
                        "Duplicado misma clase eliminado: %s %.2f < %.2f IoU=%.2f",
                        CLASES_HERRAMIENTAS[categoria],
                        confianza,
                        otro.score,
                        iou
                    );
                    break;
                }

                continue;
            }

            // Conflicto entre clases sobre el mismo objeto.
            // Si la actual es HAMMER y existe PLIER solapada con score
            // razonable, descartamos HAMMER y dejamos sobrevivir PLIER.
            if (
                categoria == 0 &&
                otra_categoria == 1 &&
                confianza <= HAMMER_MAX_EN_CONFLICTO &&
                otro.score >= PLIER_RESCATE_MIN &&
                iou >= IOU_CONFLICTO_CLASES
            ) {
                hammer_confundido_con_plier = true;

                ESP_LOGI(
                    TAG,
                    "CONFLICTO RESUELTO: hammer %.2f descartado; gana plier %.2f IoU=%.2f",
                    confianza,
                    otro.score,
                    iou
                );
                break;
            }
        }

        if (duplicada_por_iou || hammer_confundido_con_plier) {
            continue;
        }

        bool secundaria =
            false;

        for (
            const auto &otro :
            resultados
        ) {
            const int otra_categoria =
                static_cast<int>(
                    otro.category
                );

            if (
                otra_categoria != categoria ||
                otro.score < (
                    otra_categoria == 1
                        ? CONFIANZA_PLIER
                        : CONFIANZA_HAMMER
                )
            ) {
                continue;
            }

            int ox1 = 0;
            int oy1 = 0;
            int ox2 = 0;
            int oy2 = 0;

            obtener_caja(
                otro,
                ox1,
                oy1,
                ox2,
                oy2
            );

            const int oancho =
                ox2 - ox1;

            const int oalto =
                oy2 - oy1;

            if (
                oancho < 5 ||
                oalto < 5
            ) {
                continue;
            }

            const int area_otro =
                oancho * oalto;

            if (
                area_otro <
                static_cast<int>(
                    area_actual * 1.30f
                )
            ) {
                continue;
            }

            const int ix1 =
                x1 > ox1 ? x1 : ox1;

            const int iy1 =
                y1 > oy1 ? y1 : oy1;

            const int ix2 =
                x2 < ox2 ? x2 : ox2;

            const int iy2 =
                y2 < oy2 ? y2 : oy2;

            const int iw =
                ix2 > ix1
                    ? ix2 - ix1
                    : 0;

            const int ih =
                iy2 > iy1
                    ? iy2 - iy1
                    : 0;

            const int area_interseccion =
                iw * ih;

            const float fraccion_dentro =
                area_actual > 0
                    ? (
                        static_cast<float>(
                            area_interseccion
                        ) /
                        static_cast<float>(
                            area_actual
                        )
                    )
                    : 0.0f;

            if (
                fraccion_dentro >= 0.65f
            ) {
                secundaria = true;
                break;
            }
        }

        if (secundaria) {
            cajas_secundarias++;

            ESP_LOGI(
                TAG,
                "Caja secundaria eliminada: %s %.2f",
                CLASES_HERRAMIENTAS[categoria],
                confianza
            );

            continue;
        }

        detecciones_validas++;

        if (categoria == 0) {
            martillos_detectados++;
        }
        else if (categoria == 1) {
            pinzas_detectadas++;
        }

        ESP_LOGI(
            TAG,
            "DETECCION FINAL -> %s %.2f caja=[%d,%d,%d,%d]",
            CLASES_HERRAMIENTAS[categoria],
            confianza,
            x1,
            y1,
            x2,
            y2
        );

        const FirmaVisual firma =
            calcular_firma_visual(
                buffer_deteccion_rgb565,
                x1,
                y1,
                x2,
                y2
            );

        // ====================================================
        // EVITAR QUE LA MISMA HERRAMIENTA SE CUENTE DOS VECES
        // Ej.: PLIER directo + HAMMER rescatado como PLIER.
        // ====================================================

        bool visible_duplicada = false;

        for (
            int vi = 0;
            vi < cantidad_visibles;
            ++vi
        ) {
            if (
                visibles[vi].categoria !=
                categoria
            ) {
                continue;
            }

            const float iou_visible =
                calcular_iou_cajas(
                    x1,
                    y1,
                    x2,
                    y2,
                    visibles[vi].x1,
                    visibles[vi].y1,
                    visibles[vi].x2,
                    visibles[vi].y2
                );

            if (iou_visible >= 0.80f) {
                visible_duplicada = true;

                // Conservamos la confianza mas alta.
                if (
                    confianza >
                    visibles[vi].confianza
                ) {
                    visibles[vi].confianza =
                        confianza;
                    visibles[vi].x1 = x1;
                    visibles[vi].y1 = y1;
                    visibles[vi].x2 = x2;
                    visibles[vi].y2 = y2;
                    visibles[vi].firma = firma;
                }

                ESP_LOGI(
                    TAG,
                    "VISIBLE DUPLICADA eliminada: %s %.2f IoU=%.2f",
                    CLASES_HERRAMIENTAS[categoria],
                    confianza,
                    iou_visible
                );

                break;
            }
        }

        // Siempre se muestra en pantalla, pero una sola vez.
        if (
            !visible_duplicada &&
            cantidad_visibles <
            MAX_NUEVOS_POR_FRAME
        ) {
            DeteccionNueva &visible =
                visibles[
                    cantidad_visibles++
                ];

            visible.id = 0;
            visible.categoria = categoria;
            visible.x1 = x1;
            visible.y1 = y1;
            visible.x2 = x2;
            visible.y2 = y2;
            visible.confianza = confianza;
            visible.firma = firma;
        }

        // ----------------------------------------------------
        // HISTORIAL
        // ----------------------------------------------------

        const int indice_rastreo =
            buscar_rastreo_activo(
                x1,
                y1,
                x2,
                y2,
                firma,
                tiempo_ms
            );

        if (indice_rastreo >= 0) {
            actualizar_rastreo(
                indice_rastreo,
                x1,
                y1,
                x2,
                y2,
                firma,
                tiempo_ms
            );

            duplicados_rastreo++;
            continue;
        }

        const int indice_historial =
            buscar_firma_en_historial(
                firma
            );

        if (indice_historial >= 0) {
            const int id_existente =
                firmas_historial[
                    indice_historial
                ].id;

            crear_rastreo(
                id_existente,
                x1,
                y1,
                x2,
                y2,
                firma,
                tiempo_ms
            );

            firmas_historial[
                indice_historial
            ].firma = firma;

            duplicados_historial++;
            continue;
        }

        if (
            coincide_con_nuevo_del_mismo_frame(
                nuevos,
                cantidad_nuevos,
                x1,
                y1,
                x2,
                y2,
                firma
            )
        ) {
            continue;
        }

        if (
            cantidad_nuevos >=
            MAX_NUEVOS_POR_FRAME
        ) {
            ESP_LOGW(
                TAG,
                "Limite de objetos nuevos alcanzado"
            );

            break;
        }

        DeteccionNueva &nuevo =
            nuevos[
                cantidad_nuevos++
            ];

        nuevo.id = 0;
        nuevo.categoria = categoria;
        nuevo.x1 = x1;
        nuevo.y1 = y1;
        nuevo.x2 = x2;
        nuevo.y2 = y2;
        nuevo.confianza = confianza;
        nuevo.firma = firma;
    }

    // ========================================================
    // PREPARAR FOTO ANOTADA PARA WEB / HISTORIAL
    // ========================================================

    const size_t bytes_foto_web =
        static_cast<size_t>(FOTO_ANCHO) *
        static_cast<size_t>(FOTO_ALTO) *
        2U;

    uint8_t *buffer_web_rgb565 =
        static_cast<uint8_t *>(
            heap_caps_malloc(
                bytes_foto_web,
                MALLOC_CAP_SPIRAM |
                MALLOC_CAP_8BIT
            )
        );

    const uint8_t *buffer_guardado_rgb565 =
        buffer_deteccion_rgb565;

    if (buffer_web_rgb565 != nullptr) {
        memcpy(
            buffer_web_rgb565,
            buffer_deteccion_rgb565,
            bytes_foto_web
        );

        dibujar_cajas_para_web(
            buffer_web_rgb565,
            visibles,
            cantidad_visibles
        );

        buffer_guardado_rgb565 =
            buffer_web_rgb565;

        ESP_LOGI(
            TAG,
            "CAPTURA WEB: %d caja(s) dibujada(s)",
            cantidad_visibles
        );
    }
    else {
        ESP_LOGW(
            TAG,
            "Sin PSRAM para captura anotada; se guarda original"
        );
    }

    // ========================================================
    // GUARDAR NUEVOS EN HISTORIAL
    // ========================================================

    bool historial_ok =
        true;

    if (cantidad_nuevos > 0) {
        historial_ok =
            registrar_evento_nuevo(
                buffer_guardado_rgb565,
                nuevos,
                cantidad_nuevos,
                tiempo_ms
            );

        if (historial_ok) {
            bool hay_martillo_nuevo =
                false;

            for (
                int i = 0;
                i < cantidad_nuevos;
                ++i
            ) {
                if (
                    nuevos[i].categoria == 0
                ) {
                    hay_martillo_nuevo =
                        true;
                    break;
                }
            }

            if (hay_martillo_nuevo) {
                reproducir_voz_martillo();
            }
        }
    }

    const uint64_t respuesta_ms =
        ahora_ms() -
        inicio_respuesta_ms;

    // ========================================================
    // ULTIMA DETECCION PARA LA PAGINA WEB
    // Se guarda SIEMPRE que exista una deteccion valida.
    // ========================================================

    if (cantidad_visibles > 0) {
        if (
            guardar_jpeg_actual(
                buffer_guardado_rgb565,
                "/storage/ultima_deteccion.jpg"
            )
        ) {
            ESP_LOGI(
                TAG,
                "WEB FOTO ACTUALIZADA: /storage/ultima_deteccion.jpg"
            );
        }
        else {
            ESP_LOGW(
                TAG,
                "No se pudo actualizar ultima_deteccion.jpg"
            );
        }
    }


    if (cantidad_visibles > 0) {
        float max_h = 0.0f;
        float max_p = 0.0f;
        for (int i = 0; i < cantidad_visibles; ++i) {
            if (visibles[i].categoria == 0 && visibles[i].confianza > max_h) max_h = visibles[i].confianza;
            if (visibles[i].categoria == 1 && visibles[i].confianza > max_p) max_p = visibles[i].confianza;
        }
        registro_flash_guardar_evento(
            buffer_guardado_rgb565,
            FOTO_ANCHO,
            FOTO_ALTO,
            CALIDAD_JPEG,
            martillos_detectados,
            pinzas_detectadas,
            respuesta_ms,
            max_h,
            max_p
        );

        // Actualizar /api y guardar la fila de Excel.
        web_panel_actualizar_resultado(
            martillos_detectados,
            pinzas_detectadas,
            respuesta_ms,
            max_h,
            max_p
        );

        ESP_LOGI(
            TAG,
            "WEB/EXCEL ACTUALIZADO: M=%d P=%d H=%.2f P=%.2f",
            martillos_detectados,
            pinzas_detectadas,
            max_h,
            max_p
        );
    }

    if (buffer_web_rgb565 != nullptr) {
        free(buffer_web_rgb565);
        buffer_web_rgb565 = nullptr;
    }

    // ========================================================
    // DIBUJAR RESULTADO
    // ========================================================

    if (bsp_display_lock(0)) {
        vista.capa_detecciones =
            crear_capa_detecciones(
                pantalla
            );

        for (
            int i = 0;
            i < cantidad_visibles;
            ++i
        ) {
            dibujar_deteccion_nueva(
                vista.capa_detecciones,
                visibles[i]
            );
        }

        // MARTILLOS - ROJO.
        lv_obj_t *label_martillos =
            lv_label_create(
                vista.capa_detecciones
            );

        char texto_martillos[40];

        snprintf(
            texto_martillos,
            sizeof(texto_martillos),
            "MARTILLOS: %d",
            martillos_detectados
        );

        lv_label_set_text(
            label_martillos,
            texto_martillos
        );

        lv_obj_set_style_text_color(
            label_martillos,
            lv_color_hex(0xFF3030),
            LV_PART_MAIN
        );

        lv_obj_set_style_bg_color(
            label_martillos,
            lv_color_hex(0x000000),
            LV_PART_MAIN
        );

        lv_obj_set_style_bg_opa(
            label_martillos,
            LV_OPA_80,
            LV_PART_MAIN
        );

        lv_obj_set_style_pad_all(
            label_martillos,
            3,
            LV_PART_MAIN
        );

        lv_obj_set_pos(
            label_martillos,
            4,
            4
        );

        // PINZAS - CELESTE.
        lv_obj_t *label_pinzas =
            lv_label_create(
                vista.capa_detecciones
            );

        char texto_pinzas[40];

        snprintf(
            texto_pinzas,
            sizeof(texto_pinzas),
            "PINZAS: %d",
            pinzas_detectadas
        );

        lv_label_set_text(
            label_pinzas,
            texto_pinzas
        );

        lv_obj_set_style_text_color(
            label_pinzas,
            lv_color_hex(0x00D9FF),
            LV_PART_MAIN
        );

        lv_obj_set_style_bg_color(
            label_pinzas,
            lv_color_hex(0x000000),
            LV_PART_MAIN
        );

        lv_obj_set_style_bg_opa(
            label_pinzas,
            LV_OPA_80,
            LV_PART_MAIN
        );

        lv_obj_set_style_pad_all(
            label_pinzas,
            3,
            LV_PART_MAIN
        );

        lv_obj_set_pos(
            label_pinzas,
            4,
            27
        );

        // TIEMPO - AMARILLO.
        lv_obj_t *label_tiempo =
            lv_label_create(
                vista.capa_detecciones
            );

        char texto_tiempo[48];

        snprintf(
            texto_tiempo,
            sizeof(texto_tiempo),
            "RESPUESTA: %llu ms",
            static_cast<unsigned long long>(
                respuesta_ms
            )
        );

        lv_label_set_text(
            label_tiempo,
            texto_tiempo
        );

        lv_obj_set_style_text_color(
            label_tiempo,
            lv_color_hex(0xFFFF00),
            LV_PART_MAIN
        );

        lv_obj_set_style_bg_color(
            label_tiempo,
            lv_color_hex(0x000000),
            LV_PART_MAIN
        );

        lv_obj_set_style_bg_opa(
            label_tiempo,
            LV_OPA_80,
            LV_PART_MAIN
        );

        lv_obj_set_style_pad_all(
            label_tiempo,
            3,
            LV_PART_MAIN
        );

        lv_obj_set_pos(
            label_tiempo,
            4,
            50
        );

        char texto_estado[64];

        if (!historial_ok) {
            snprintf(
                texto_estado,
                sizeof(texto_estado),
                "DETECTADO | ERROR HISTORIAL"
            );

            lv_obj_set_style_text_color(
                etiqueta_estado,
                lv_color_hex(0xFF3030),
                LV_PART_MAIN
            );
        }
        else if (cantidad_visibles > 0) {
            snprintf(
                texto_estado,
                sizeof(texto_estado),
                "RESULTADO | NUEVOS: %d",
                cantidad_nuevos
            );

            lv_obj_set_style_text_color(
                etiqueta_estado,
                lv_color_hex(0xFFFFFF),
                LV_PART_MAIN
            );
        }
        else {
            snprintf(
                texto_estado,
                sizeof(texto_estado),
                "SIN HERRAMIENTAS"
            );

            lv_obj_set_style_text_color(
                etiqueta_estado,
                lv_color_hex(0xFFFF00),
                LV_PART_MAIN
            );
        }

        lv_label_set_text(
            etiqueta_estado,
            texto_estado
        );

        lv_obj_move_foreground(
            etiqueta_estado
        );

        if (etiqueta_fps != nullptr) {
            lv_obj_move_foreground(
                etiqueta_fps
            );
        }

        bsp_display_unlock();
    }

    ESP_LOGI(
        TAG,
        "RESULTADO: martillos=%d pinzas=%d tiempo=%llums validas=%d cajas_secundarias=%d nuevas_hist=%d",
        martillos_detectados,
        pinzas_detectadas,
        static_cast<unsigned long long>(
            respuesta_ms
        ),
        detecciones_validas,
        cajas_secundarias,
        cantidad_nuevos
    );

    return true;
}

// ============================================================
// TAREA YOLO EN EL SEGUNDO NUCLEO
// ============================================================

struct ContextoDeteccion {
    lv_obj_t *pantalla = nullptr;
    lv_obj_t *etiqueta_estado = nullptr;
    VistaCamara *vista = nullptr;
    dl::Model *modelo = nullptr;
    YOLO26 *detector = nullptr;
};

static void tarea_deteccion_yolo(
    void *parametro
)
{
    ContextoDeteccion *contexto =
        static_cast<ContextoDeteccion *>(
            parametro
        );

    // Dejamos arrancar el video primero.
    vTaskDelay(
        pdMS_TO_TICKS(800)
    );

    while (true) {
        detectar_automaticamente(
            contexto->pantalla,
            contexto->etiqueta_estado,
            *contexto->vista,
            contexto->modelo,
            contexto->detector
        );

        // Pausa real después de cada inferencia para permitir
        // que CPU1 ejecute tareas Idle/sistema.
        vTaskDelay(
            pdMS_TO_TICKS(
                INTERVALO_DETECCION_MS
            )
        );
    }
}

// ============================================================
// MAIN
// ============================================================

extern "C" void app_main(void)
{
    ESP_LOGI(
        TAG,
        "Inicio: video + deteccion MANUAL por botones + historial"
    );

    // --------------------------------------------------------
    // 1. I2C + pantalla
    // --------------------------------------------------------

    ESP_ERROR_CHECK(
        bsp_i2c_init()
    );

    lv_display_t *display =
        bsp_display_start();

    if (display == nullptr) {
        ESP_LOGE(
            TAG,
            "No se pudo iniciar pantalla"
        );

        return;
    }

    bsp_display_brightness_set(
        80
    );

    lv_obj_t *pantalla = nullptr;
    lv_obj_t *etiqueta_estado = nullptr;

    if (bsp_display_lock(0)) {
        pantalla =
            lv_display_get_screen_active(
                display
            );

        lv_obj_set_style_bg_color(
            pantalla,
            lv_color_hex(0x000000),
            LV_PART_MAIN
        );

        lv_obj_set_style_bg_opa(
            pantalla,
            LV_OPA_COVER,
            LV_PART_MAIN
        );

        etiqueta_estado =
            lv_label_create(
                pantalla
            );

        lv_label_set_text(
            etiqueta_estado,
            "INICIANDO..."
        );

        lv_obj_set_style_text_color(
            etiqueta_estado,
            lv_color_hex(0xFFFFFF),
            LV_PART_MAIN
        );

        lv_obj_set_style_bg_color(
            etiqueta_estado,
            lv_color_hex(0x000000),
            LV_PART_MAIN
        );

        lv_obj_set_style_bg_opa(
            etiqueta_estado,
            LV_OPA_80,
            LV_PART_MAIN
        );

        lv_obj_set_style_pad_all(
            etiqueta_estado,
            4,
            LV_PART_MAIN
        );

        lv_obj_align(
            etiqueta_estado,
            LV_ALIGN_TOP_MID,
            0,
            3
        );

        etiqueta_fps =
            lv_label_create(
                pantalla
            );

        lv_label_set_text(
            etiqueta_fps,
            "FPS: 0.0"
        );

        lv_obj_set_style_text_color(
            etiqueta_fps,
            lv_color_hex(0x00FFFF),
            LV_PART_MAIN
        );

        lv_obj_set_style_bg_color(
            etiqueta_fps,
            lv_color_hex(0x000000),
            LV_PART_MAIN
        );

        lv_obj_set_style_bg_opa(
            etiqueta_fps,
            LV_OPA_70,
            LV_PART_MAIN
        );

        lv_obj_set_style_pad_all(
            etiqueta_fps,
            3,
            LV_PART_MAIN
        );

        lv_obj_align(
            etiqueta_fps,
            LV_ALIGN_BOTTOM_RIGHT,
            -4,
            -4
        );

        bsp_display_unlock();
    }

    // --------------------------------------------------------
    // 2. POWER
    // --------------------------------------------------------

    if (
        iniciar_boton_power() !=
        ESP_OK
    ) {
        actualizar_estado(
            etiqueta_estado,
            "ERROR EN POWER",
            0xFF0000
        );

        return;
    }

    // --------------------------------------------------------
    // 3. STORAGE + HISTORIAL
    // --------------------------------------------------------

    actualizar_estado(
        etiqueta_estado,
        "CARGANDO HISTORIAL...",
        0xFFFF00
    );

    if (!iniciar_storage()) {
        actualizar_estado(
            etiqueta_estado,
            "ERROR EN STORAGE",
            0xFF0000
        );

        return;
    }

    resetear_datos_iniciales_una_vez();

    if (!cargar_historial()) {
        actualizar_estado(
            etiqueta_estado,
            "ERROR EN HISTORIAL",
            0xFF0000
        );

        return;
    }

    registro_flash_iniciar();

    // --------------------------------------------------------
    // 4. CAMARA
    // --------------------------------------------------------

    actualizar_estado(
        etiqueta_estado,
        "INICIANDO CAMARA...",
        0xFFFF00
    );

    camera_config_t configuracion_camara =
        BSP_CAMERA_DEFAULT_CONFIG;

    configuracion_camara.pixel_format =
        PIXFORMAT_RGB565;

    configuracion_camara.frame_size =
        FRAMESIZE_QVGA;

    // Un framebuffer reduce presión sobre PSRAM y evita
    // acumulación de cuadros mientras YOLO está ocupado.
    configuracion_camara.fb_count = 1;

    configuracion_camara.fb_location =
        CAMERA_FB_IN_PSRAM;

    configuracion_camara.grab_mode =
        CAMERA_GRAB_WHEN_EMPTY;

    const esp_err_t error_camara =
        esp_camera_init(
            &configuracion_camara
        );

    if (error_camara != ESP_OK) {
        ESP_LOGE(
            TAG,
            "No se pudo iniciar camara: %s",
            esp_err_to_name(
                error_camara
            )
        );

        actualizar_estado(
            etiqueta_estado,
            "ERROR EN CAMARA",
            0xFF0000
        );

        return;
    }

    sensor_t *sensor =
        esp_camera_sensor_get();

    if (sensor != nullptr) {
        sensor->set_vflip(
            sensor,
            BSP_CAMERA_VFLIP
        );

        sensor->set_hmirror(
            sensor,
            BSP_CAMERA_HMIRROR
        );
    }

    // Estabilizar sensor.
    for (int i = 0; i < 3; ++i) {
        camera_fb_t *frame =
            esp_camera_fb_get();

        if (frame != nullptr) {
            esp_camera_fb_return(
                frame
            );
        }

        vTaskDelay(
            pdMS_TO_TICKS(30)
        );
    }

    // --------------------------------------------------------
    // 5. PREPARAR PARALELISMO VIDEO / YOLO
    // --------------------------------------------------------

    mutex_camara =
        xSemaphoreCreateMutex();

    if (mutex_camara == nullptr) {
        actualizar_estado(
            etiqueta_estado,
            "ERROR MUTEX CAMARA",
            0xFF0000
        );

        return;
    }

    const size_t bytes_deteccion_rgb565 =
        static_cast<size_t>(
            FOTO_ANCHO
        ) *
        static_cast<size_t>(
            FOTO_ALTO
        ) *
        2U;

    const size_t bytes_deteccion_rgb888 =
        static_cast<size_t>(
            FOTO_ANCHO
        ) *
        static_cast<size_t>(
            FOTO_ALTO
        ) *
        3U;

    buffer_deteccion_rgb565 =
        static_cast<uint8_t *>(
            heap_caps_malloc(
                bytes_deteccion_rgb565,
                MALLOC_CAP_SPIRAM |
                MALLOC_CAP_8BIT
            )
        );

    buffer_deteccion_rgb888 =
        static_cast<uint8_t *>(
            heap_caps_malloc(
                bytes_deteccion_rgb888,
                MALLOC_CAP_SPIRAM |
                MALLOC_CAP_8BIT
            )
        );

    if (
        buffer_deteccion_rgb565 == nullptr ||
        buffer_deteccion_rgb888 == nullptr
    ) {
        actualizar_estado(
            etiqueta_estado,
            "ERROR MEMORIA DETECCION",
            0xFF0000
        );

        return;
    }

    ESP_LOGI(
        TAG,
        "Video y YOLO preparados para trabajar en paralelo"
    );

    registrar_memoria("BUFFERS_CREADOS");


    // Wi-Fi reserva su memoria DESPUES de la camara
    // y ANTES de cargar el modelo YOLO.
    ESP_LOGI(
        TAG,
        "INICIANDO WIFI TEMPRANO (antes de cargar YOLO)"
    );

    if (!web_panel_iniciar_wifi_temprano()) {
        ESP_LOGW(
            TAG,
            "SoftAP no disponible; detector continua sin web"
        );
    }

    // --------------------------------------------------------
    // 6. VIDEO
    // --------------------------------------------------------

    VistaCamara vista;

    if (
        !iniciar_vista_video(
            pantalla,
            etiqueta_estado,
            vista
        )
    ) {
        actualizar_estado(
            etiqueta_estado,
            "ERROR EN VIDEO",
            0xFF0000
        );

        return;
    }

    mostrar_video_un_frame(
        vista
    );

    // --------------------------------------------------------
    // 7. YOLO26
    // --------------------------------------------------------

    actualizar_estado(
        etiqueta_estado,
        "CARGANDO YOLO26...",
        0xFFFF00
    );

    const esp_partition_t *particion_modelo =
        esp_partition_find_first(
            ESP_PARTITION_TYPE_DATA,
            ESP_PARTITION_SUBTYPE_DATA_SPIFFS,
            "model"
        );

    if (particion_modelo == nullptr) {
        actualizar_estado(
            etiqueta_estado,
            "ERROR: SIN MODELO",
            0xFF0000
        );

        return;
    }

    dl::Model *modelo =
        new dl::Model(
            "model",
            fbs::MODEL_LOCATION_IN_FLASH_PARTITION,
            0,
            dl::MEMORY_MANAGER_GREEDY,
            nullptr,
            false
        );

    if (modelo == nullptr) {
        actualizar_estado(
            etiqueta_estado,
            "ERROR AL CARGAR MODELO",
            0xFF0000
        );

        return;
    }

    YOLO26 *detector =
        new YOLO26(
            modelo,
            YOLO_TARGET_K,
            CONFIANZA_ENTRADA_YOLO,
            CLASES_HERRAMIENTAS
        );

    if (detector == nullptr) {
        delete modelo;

        actualizar_estado(
            etiqueta_estado,
            "ERROR EN YOLO26",
            0xFF0000
        );

        return;
    }

    ESP_LOGI(
        TAG,
        "YOLO26 cargado correctamente: clases=2, input=256x256"
    );

    registrar_memoria("YOLO_CARGADO");



    // --------------------------------------------------------
    // 8. AUDIO
    // --------------------------------------------------------

    ESP_LOGI(TAG, "AUDIO DESACTIVADO temporalmente para liberar RAM WiFi");
    // Ignorar el clic usado para encender.
    axp2101_escribir_registro(
        AXP2101_IRQ_ESTADO_1,
        AXP2101_MASCARA_POWER
    );

    vTaskDelay(
        pdMS_TO_TICKS(400)
    );

    mostrar_estado_video(
        etiqueta_estado
    );

    ESP_LOGI(
        TAG,
        "Sistema listo. Historial actual=%d",
        ultimo_id_historial
    );


    ESP_LOGI(
        TAG,
        "INICIANDO SERVIDOR HTTP"
    );

    if (!web_panel_iniciar_http_tarde()) {
        ESP_LOGW(
            TAG,
            "HTTP no disponible; botones siguen activos"
        );
    }
else {
        ESP_LOGI(
            TAG,
            "WIFI/WEB INICIADO CORRECTAMENTE"
        );
    }

    ESP_LOGI(
        TAG,
        "Modo manual listo: ROJO=detectar | AZUL=video"
    );

    // --------------------------------------------------------
    // 9. BUCLE PRINCIPAL: VIDEO + POWER + FPS
    // --------------------------------------------------------

    uint64_t ultimo_calculo_fps_ms =
        ahora_ms();

    uint32_t frames_ultimo_calculo =
        contador_frames_video;

    bool rojo_anterior = false;
    bool azul_anterior = false;
    bool modo_resultado = false;
    bool procesando_yolo = false;

    uint32_t ultimo_diag_wifi_ms = 0;

    while (true) {
        const uint8_t evento =
            leer_evento_power();

        if (evento & AXP2101_EVENTO_LARGO) {
            ESP_LOGI(TAG, "POWER largo");
        }
        else if (evento & AXP2101_EVENTO_CORTO) {
            char texto[64];

            snprintf(
                texto,
                sizeof(texto),
                "HISTORIAL: %d",
                ultimo_id_historial
            );

            actualizar_estado(
                etiqueta_estado,
                texto,
                0x00FFFF
            );

            ESP_LOGI(
                TAG,
                "Historial total: %d",
                ultimo_id_historial
            );
        }

        const bool rojo =
            boton_rojo_pulsado();

        const bool azul =
            boton_azul_pulsado();

        // ROJO: captura + YOLO + resultado + historial.
        if (
            rojo &&
            !rojo_anterior &&
            !procesando_yolo
        ) {
            procesando_yolo = true;
            modo_resultado = true;

            registrar_memoria("ANTES_DETECCION");

            ESP_LOGI(
                TAG,
                "BOTON ROJO: capturando y ejecutando YOLO..."
            );

            actualizar_estado(
                etiqueta_estado,
                "CAPTURANDO...",
                0xFFFF00
            );

            const bool ok =
                detectar_automaticamente(
                    pantalla,
                    etiqueta_estado,
                    vista,
                    modelo,
                    detector
                );

            if (!ok) {
                actualizar_estado(
                    etiqueta_estado,
                    "ERROR EN DETECCION",
                    0xFF0000
                );

                ESP_LOGE(
                    TAG,
                    "BOTON ROJO: deteccion fallo"
                );
            }
            else {
                ESP_LOGI(
                    TAG,
                    "BOTON ROJO: deteccion terminada"
                );

                ESP_LOGI(
                    TAG,
                    "RESULTADO CONGELADO: pulse AZUL para volver al video"
                );

                modo_resultado = true;
            }

            registrar_memoria("DESPUES_DETECCION");

            procesando_yolo = false;
        }

        // AZUL: borrar detecciones y volver a video.
        if (
            azul &&
            !azul_anterior &&
            !procesando_yolo
        ) {
            ESP_LOGI(
                TAG,
                "BOTON AZUL: regresar a VIDEO"
            );

            borrar_detecciones(
                vista
            );

            modo_resultado = false;

            mostrar_estado_video(
                etiqueta_estado
            );

            mostrar_video_un_frame(
                vista
            );
        }

        rojo_anterior = rojo;
        azul_anterior = azul;

        if (
            !modo_resultado &&
            !procesando_yolo
        ) {
            mostrar_video_un_frame(
                vista
            );
        }

        const uint64_t fps_ahora_ms =
            ahora_ms();

        if (
            fps_ahora_ms -
            ultimo_calculo_fps_ms >=
            1000
        ) {
            const uint32_t frames_actuales =
                contador_frames_video;

            const uint32_t diferencia_frames =
                frames_actuales -
                frames_ultimo_calculo;

            const uint64_t diferencia_ms =
                fps_ahora_ms -
                ultimo_calculo_fps_ms;

            const float fps =
                diferencia_ms > 0
                    ? (
                        static_cast<float>(
                            diferencia_frames
                        ) *
                        1000.0f /
                        static_cast<float>(
                            diferencia_ms
                        )
                    )
                    : 0.0f;

            frames_ultimo_calculo =
                frames_actuales;

            ultimo_calculo_fps_ms =
                fps_ahora_ms;

            if (
                etiqueta_fps != nullptr &&
                bsp_display_lock(0)
            ) {
                char texto_fps[32];

                snprintf(
                    texto_fps,
                    sizeof(texto_fps),
                    "FPS: %.1f",
                    fps
                );

                lv_label_set_text(
                    etiqueta_fps,
                    texto_fps
                );

                lv_obj_move_foreground(
                    etiqueta_fps
                );

                bsp_display_unlock();
            }
        }

        const uint32_t ahora_diag_wifi =
            static_cast<uint32_t>(
                esp_timer_get_time() / 1000ULL
            );

        if (
            ahora_diag_wifi -
            ultimo_diag_wifi_ms >=
            10000
        ) {
            ultimo_diag_wifi_ms =
                ahora_diag_wifi;

            web_panel_diagnostico_wifi();
        }

        vTaskDelay(
            pdMS_TO_TICKS(50)
        );
    }
}

















