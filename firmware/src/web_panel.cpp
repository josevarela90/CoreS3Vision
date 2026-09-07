#include "web_panel.h"
#include "project_config.h"

#include <atomic>
#include <dirent.h>
#include <inttypes.h>
#include <stdio.h>
#include <string.h>
#include <sys/stat.h>

#include "esp_event.h"
#include "esp_http_server.h"
#include "esp_log.h"
#include "esp_netif.h"
#include "esp_timer.h"
#include "esp_wifi.h"
#include "esp_heap_caps.h"
#include "nvs_flash.h"

static const char *TAG = "WEB_PANEL";
static const char *WIFI_SSID = PROJECT_WIFI_SSID;
static const char *WIFI_PASS = PROJECT_WIFI_PASS;
static const char *CSV_PATH = "/storage/detecciones_excel.csv";

static std::atomic_bool captura(false);
static std::atomic_int mh(0), pp(0), ch(0), cp(0);
static std::atomic_uint_fast64_t ms(0);
static httpd_handle_t server = nullptr;
static bool panel_iniciado = false;
static bool wifi_temprano_iniciado = false;
static bool http_tarde_iniciado = false;

static const char PAGE[] =
"<!doctype html><html lang='es'><head><meta charset='utf-8'>"
"<meta name='viewport' content='width=device-width,initial-scale=1'><link rel='icon' href='data:,'>"
"<title>CoreS3 Herramientas</title>"
"<style>"
"body{font-family:Arial;background:#101214;color:#eee;margin:0;padding:18px}"
".wrap{max-width:900px;margin:auto}.c{background:#1a1e22;border-radius:14px;padding:14px;margin:12px 0}"
"button,a{display:inline-block;border:0;border-radius:10px;padding:13px 17px;margin:4px;font-size:16px;text-decoration:none}"
".r{background:#d93025;color:#fff}.g{background:#188038;color:#fff}.h{color:#ff4747}.p{color:#00d9ff}.t{color:#ffe642}"
".grid{display:grid;grid-template-columns:repeat(auto-fill,minmax(220px,1fr));gap:10px}"
"img{width:100%;border-radius:8px}.item{background:#1a1e22;border-radius:12px;padding:8px}small{color:#aaa}"
"</style></head><body><div class='wrap'>"
"<h1>CoreS3 - Detector de herramientas</h1>"
"<div class='c'>"
"<button class='r' onclick='cap()'>Tomar foto y analizar</button>"
"<a class='g' href='/datos.csv'>Descargar datos para Excel</a>"
"<p>ROJO fisico = detectar | AZUL fisico = video</p>"
"</div>"
"<div id='e' class='c'>Cargando...</div>"
"<h2>Última detección</h2><div id='g' class='grid'></div>"
"</div>"
"<script>"
"async function cap(){await fetch('/capturar',{method:'POST'});document.getElementById('e').innerText='Captura solicitada';setTimeout(up,13000);setTimeout(up,16000);setTimeout(up,20000)}"
"async function up(){try{let r=await fetch('/api?x='+Date.now());let j=await r.json();"
"document.getElementById('e').innerHTML='<span class=h>Martillos: '+j.martillos+'</span> | <span class=p>Pinzas: '+j.pinzas+'</span> | <span class=t>Respuesta: '+j.ms+' ms</span><br>Conf. martillo: '+Math.round(j.ch*100)+'% | Conf. pinza: '+Math.round(j.cp*100)+'%';"
"document.getElementById('g').innerHTML=j.fotos.map(x=>'<div class=item><img loading=\"eager\" style=\"display:block;width:100%;height:300px;object-fit:contain;background:#111;border-radius:10px;margin-bottom:8px\" src=\"/foto?name='+encodeURIComponent(x)+'&v='+Date.now()+'\"><small>'+x+'</small></div>').join('');"
"}catch(e){document.getElementById('e').innerText='Sin conexion'}}up();"
"</script></body></html>";

static bool asegurar_csv()
{
    struct stat st = {};
    if (stat(CSV_PATH, &st) == 0) return true;

    FILE *f = fopen(CSV_PATH, "w");
    if (!f) return false;

    fprintf(
        f,
        "sep=;\r\nid;uptime_ms;martillos;pinzas;respuesta_ms;confianza_martillo;confianza_pinza\r\n"
    );
    fclose(f);
    return true;
}

static void guardar_csv(
    int martillos,
    int pinzas,
    uint64_t respuesta_ms,
    float conf_h,
    float conf_p
)
{
    if (martillos <= 0 && pinzas <= 0) return;
    if (!asegurar_csv()) return;

    FILE *f = fopen(CSV_PATH, "a");
    if (!f) return;

    uint64_t id =
        static_cast<uint64_t>(
            esp_timer_get_time() / 1000ULL
        );

    fprintf(
        f,
        "%" PRIu64 ";%" PRIu64 ";%d;%d;%" PRIu64 ";%.3f;%.3f\r\n",
        id,
        id,
        martillos,
        pinzas,
        respuesta_ms,
        conf_h,
        conf_p
    );

    fflush(f);
    fclose(f);

    ESP_LOGI(
        TAG,
        "CSV guardado: M=%d P=%d tiempo=%" PRIu64 "ms",
        martillos,
        pinzas,
        respuesta_ms
    );
}

static bool iniciar_sta()
{
    ESP_LOGI(
        TAG,
        "========================================"
    );
    ESP_LOGI(
        TAG,
        "INICIANDO WIFI EN MODO STA"
    );
    ESP_LOGI(
        TAG,
        "RED: %s",
        WIFI_SSID
    );

    esp_err_t e =
        nvs_flash_init();

    if (
        e == ESP_ERR_NVS_NO_FREE_PAGES ||
        e == ESP_ERR_NVS_NEW_VERSION_FOUND
    ) {
        ESP_ERROR_CHECK(
            nvs_flash_erase()
        );

        e =
            nvs_flash_init();
    }

    if (e != ESP_OK) {
        ESP_LOGE(
            TAG,
            "NVS fallo: %s",
            esp_err_to_name(e)
        );

        return false;
    }

    e =
        esp_netif_init();

    if (
        e != ESP_OK &&
        e != ESP_ERR_INVALID_STATE
    ) {
        ESP_LOGE(
            TAG,
            "NETIF fallo: %s",
            esp_err_to_name(e)
        );

        return false;
    }

    e =
        esp_event_loop_create_default();

    if (
        e != ESP_OK &&
        e != ESP_ERR_INVALID_STATE
    ) {
        ESP_LOGE(
            TAG,
            "EVENT LOOP fallo: %s",
            esp_err_to_name(e)
        );

        return false;
    }

    esp_netif_t *sta =
        esp_netif_get_handle_from_ifkey(
            "WIFI_STA_DEF"
        );

    if (sta == nullptr) {
        sta =
            esp_netif_create_default_wifi_sta();
    }

    if (sta == nullptr) {
        ESP_LOGE(
            TAG,
            "No se pudo crear WIFI_STA_DEF"
        );

        return false;
    }

    wifi_init_config_t init =
        WIFI_INIT_CONFIG_DEFAULT();

    // Evita volver a consumir NVS del driver WiFi.
    init.nvs_enable = 0;

    e =
        esp_wifi_init(
            &init
        );

    if (
        e != ESP_OK &&
        e != ESP_ERR_WIFI_INIT_STATE
    ) {
        ESP_LOGE(
            TAG,
            "WIFI INIT fallo: %s",
            esp_err_to_name(e)
        );

        return false;
    }

    wifi_config_t cfg = {};

    strlcpy(
        reinterpret_cast<char *>(
            cfg.sta.ssid
        ),
        WIFI_SSID,
        sizeof(
            cfg.sta.ssid
        )
    );

    strlcpy(
        reinterpret_cast<char *>(
            cfg.sta.password
        ),
        WIFI_PASS,
        sizeof(
            cfg.sta.password
        )
    );

    cfg.sta.threshold.authmode =
        WIFI_AUTH_WPA2_PSK;

    cfg.sta.pmf_cfg.capable =
        true;

    cfg.sta.pmf_cfg.required =
        false;

    ESP_ERROR_CHECK(
        esp_wifi_set_mode(
            WIFI_MODE_STA
        )
    );

    ESP_ERROR_CHECK(
        esp_wifi_set_config(
            WIFI_IF_STA,
            &cfg
        )
    );

    ESP_ERROR_CHECK(
        esp_wifi_start()
    );

    ESP_ERROR_CHECK(
        esp_wifi_set_ps(
            WIFI_PS_NONE
        )
    );

    ESP_LOGI(
        TAG,
        "WiFi Power Save OFF para mayor estabilidad"
    );

    e =
        esp_wifi_connect();

    if (
        e != ESP_OK &&
        e != ESP_ERR_WIFI_CONN
    ) {
        ESP_LOGE(
            TAG,
            "WIFI CONNECT fallo: %s",
            esp_err_to_name(e)
        );

        return false;
    }

    ESP_LOGI(
        TAG,
        "CoreS3 intentando conectarse a %s",
        WIFI_SSID
    );

    ESP_LOGI(
        TAG,
        "========================================"
    );

    return true;
}

static esp_err_t root(httpd_req_t *r)
{
    httpd_resp_set_type(
        r,
        "text/html; charset=utf-8"
    );
    return httpd_resp_send(
        r,
        PAGE,
        HTTPD_RESP_USE_STRLEN
    );
}

static esp_err_t capt(httpd_req_t *r)
{
    captura.store(true);
    return httpd_resp_sendstr(
        r,
        "Captura solicitada"
    );
}

static bool nombre_seguro(const char *n)
{
    return n &&
           n[0] &&
           !strstr(n, "..") &&
           !strchr(n, '/') &&
           !strchr(n, '\\') &&
           strstr(n, ".jpg");
}

static esp_err_t foto(httpd_req_t *r)
{
    ESP_LOGI("WEB_PANEL", "FOTO DEBUG: handler iniciado");
    char q[180] = {};
    char n[100] = {};

    if (
        httpd_req_get_url_query_str(
            r,
            q,
            sizeof(q)
        ) != ESP_OK ||
        httpd_query_key_value(
            q,
            "name",
            n,
            sizeof(n)
        ) != ESP_OK ||
        !nombre_seguro(n)
    ) {
        return httpd_resp_send_err(
            r,
            HTTPD_400_BAD_REQUEST,
            "Nombre invalido"
        );
    }

    char p[180];

    snprintf(
        p,
        sizeof(p),
        "/storage/%s",
        n
    );

    ESP_LOGI("WEB_PANEL", "FOTO DEBUG: abriendo %s", p);

    FILE *f = fopen(p, "rb");

    ESP_LOGI("WEB_PANEL", "FOTO DEBUG: fopen terminado ptr=%p", f);

    if (!f) {
        return httpd_resp_send_err(
            r,
            HTTPD_404_NOT_FOUND,
            "No encontrada"
        );
    }

    if (fseek(f, 0, SEEK_END) != 0) {
        fclose(f);

        return httpd_resp_send_err(
            r,
            HTTPD_500_INTERNAL_SERVER_ERROR,
            "Error midiendo imagen"
        );
    }

    const long tamano_archivo = ftell(f);

    ESP_LOGI(
        "WEB_PANEL",
        "FOTO DEBUG: tamano=%ld",
        tamano_archivo
    );

    if (
        tamano_archivo <= 0 ||
        tamano_archivo > 512 * 1024
    ) {
        fclose(f);

        return httpd_resp_send_err(
            r,
            HTTPD_500_INTERNAL_SERVER_ERROR,
            "Tamano de imagen invalido"
        );
    }

    rewind(f);

    // ========================================================
    // ENVIO JPEG COMPLETO
    // Leemos primero todo el archivo desde SPIFFS hacia PSRAM
    // y realizamos un solo envio HTTP.
    // ========================================================

    uint8_t *jpeg_completo =
        static_cast<uint8_t *>(
            heap_caps_malloc(
                static_cast<size_t>(tamano_archivo),
                MALLOC_CAP_SPIRAM |
                MALLOC_CAP_8BIT
            )
        );

    if (jpeg_completo == nullptr) {
        fclose(f);

        return httpd_resp_send_err(
            r,
            HTTPD_500_INTERNAL_SERVER_ERROR,
            "Sin PSRAM para foto"
        );
    }

    const size_t leidos =
        fread(
            jpeg_completo,
            1,
            static_cast<size_t>(tamano_archivo),
            f
        );

    fclose(f);

    if (
        leidos !=
        static_cast<size_t>(tamano_archivo)
    ) {
        free(jpeg_completo);

        ESP_LOGE(
            "WEB_PANEL",
            "FOTO ERROR: lectura incompleta %u/%ld",
            static_cast<unsigned>(leidos),
            tamano_archivo
        );

        return httpd_resp_send_err(
            r,
            HTTPD_500_INTERNAL_SERVER_ERROR,
            "Lectura JPEG incompleta"
        );
    }

    httpd_resp_set_type(
        r,
        "image/jpeg"
    );

    httpd_resp_set_hdr(
        r,
        "Cache-Control",
        "no-store, no-cache, must-revalidate"
    );

    ESP_LOGI(
        "WEB_PANEL",
        "FOTO DEBUG: enviando JPEG completo %ld bytes",
        tamano_archivo
    );

    const esp_err_t resultado =
        httpd_resp_send(
            r,
            reinterpret_cast<const char *>(jpeg_completo),
            static_cast<ssize_t>(tamano_archivo)
        );

    free(jpeg_completo);

    if (resultado == ESP_OK) {
        ESP_LOGI(
            "WEB_PANEL",
            "FOTO: envio completo %ld/%ld bytes",
            tamano_archivo,
            tamano_archivo
        );
    }
    else {
        ESP_LOGE(
            "WEB_PANEL",
            "FOTO ERROR: httpd_resp_send fallo: %s",
            esp_err_to_name(resultado)
        );
    }

    return resultado;
}


// ============================================================
// HISTORIAL OFICIAL PARA SINCRONIZACION CON PC
// ============================================================

static esp_err_t eventos_csv_pc(httpd_req_t *r)
{
    const char *ruta = "/storage/eventos.csv";

    FILE *f = fopen(ruta, "rb");

    if (!f) {
        return httpd_resp_send_err(
            r,
            HTTPD_404_NOT_FOUND,
            "eventos.csv no encontrado"
        );
    }

    httpd_resp_set_type(
        r,
        "text/csv; charset=utf-8"
    );

    httpd_resp_set_hdr(
        r,
        "Cache-Control",
        "no-store, no-cache, must-revalidate"
    );

    char buffer[1024];

    while (true) {
        const size_t n =
            fread(
                buffer,
                1,
                sizeof(buffer),
                f
            );

        if (n == 0) {
            break;
        }

        esp_err_t err =
            httpd_resp_send_chunk(
                r,
                buffer,
                n
            );

        if (err != ESP_OK) {
            fclose(f);
            return err;
        }
    }

    fclose(f);

    return httpd_resp_send_chunk(
        r,
        nullptr,
        0
    );
}


// ============================================================
// DESCARGA DE UNA FOTO DEL HISTORIAL
// /captura_historial?name=captura_XXXXXXXXXX.jpg
// ============================================================

static esp_err_t captura_historial_pc(httpd_req_t *r)
{
    char query[180] = {};
    char nombre[100] = {};

    if (
        httpd_req_get_url_query_str(
            r,
            query,
            sizeof(query)
        ) != ESP_OK ||
        httpd_query_key_value(
            query,
            "name",
            nombre,
            sizeof(nombre)
        ) != ESP_OK ||
        !nombre_seguro(nombre)
    ) {
        return httpd_resp_send_err(
            r,
            HTTPD_400_BAD_REQUEST,
            "Nombre invalido"
        );
    }

    // Aceptar solamente archivos captura_*.jpg
    if (
        strncmp(
            nombre,
            "captura_",
            8
        ) != 0 ||
        strstr(
            nombre,
            ".jpg"
        ) == nullptr
    ) {
        return httpd_resp_send_err(
            r,
            HTTPD_400_BAD_REQUEST,
            "Archivo no permitido"
        );
    }

    char ruta[220];

    snprintf(
        ruta,
        sizeof(ruta),
        "/storage/%s",
        nombre
    );

    FILE *f = fopen(ruta, "rb");

    if (!f) {
        return httpd_resp_send_err(
            r,
            HTTPD_404_NOT_FOUND,
            "Captura no encontrada"
        );
    }

    if (fseek(f, 0, SEEK_END) != 0) {
        fclose(f);

        return httpd_resp_send_err(
            r,
            HTTPD_500_INTERNAL_SERVER_ERROR,
            "Error midiendo captura"
        );
    }

    const long tamano = ftell(f);

    if (
        tamano <= 0 ||
        tamano > 512 * 1024
    ) {
        fclose(f);

        return httpd_resp_send_err(
            r,
            HTTPD_500_INTERNAL_SERVER_ERROR,
            "Tamano invalido"
        );
    }

    rewind(f);

    uint8_t *jpeg =
        static_cast<uint8_t *>(
            heap_caps_malloc(
                static_cast<size_t>(tamano),
                MALLOC_CAP_SPIRAM |
                MALLOC_CAP_8BIT
            )
        );

    if (jpeg == nullptr) {
        fclose(f);

        return httpd_resp_send_err(
            r,
            HTTPD_500_INTERNAL_SERVER_ERROR,
            "Sin PSRAM"
        );
    }

    const size_t leidos =
        fread(
            jpeg,
            1,
            static_cast<size_t>(tamano),
            f
        );

    fclose(f);

    if (
        leidos !=
        static_cast<size_t>(tamano)
    ) {
        free(jpeg);

        return httpd_resp_send_err(
            r,
            HTTPD_500_INTERNAL_SERVER_ERROR,
            "Lectura incompleta"
        );
    }

    httpd_resp_set_type(
        r,
        "image/jpeg"
    );

    httpd_resp_set_hdr(
        r,
        "Cache-Control",
        "no-store, no-cache, must-revalidate"
    );

    const esp_err_t resultado =
        httpd_resp_send(
            r,
            reinterpret_cast<const char *>(jpeg),
            static_cast<ssize_t>(tamano)
        );

    free(jpeg);

    return resultado;
}


static esp_err_t api(httpd_req_t *r)
{
    char *j = static_cast<char *>(heap_caps_malloc(1536, MALLOC_CAP_SPIRAM | MALLOC_CAP_8BIT)); if (j == nullptr) return httpd_resp_send_err(r, HTTPD_500_INTERNAL_SERVER_ERROR, "Sin memoria para API");

    int pos =
        snprintf(j, 1536,
            "{\"martillos\":%d,\"pinzas\":%d,\"ms\":%llu,\"ch\":%.3f,\"cp\":%.3f,\"fotos\":[",
            mh.load(),
            pp.load(),
            (unsigned long long)ms.load(),
            ch.load() / 1000.0f,
            cp.load() / 1000.0f
        );

    struct stat ultima_info = {};

    if (
        stat(
            "/storage/ultima_deteccion.jpg",
            &ultima_info
        ) == 0
    ) {
        pos +=
            snprintf(
                j + pos,
                1536 - pos,
                "\"ultima_deteccion.jpg\""
            );
    }

    snprintf(
        j + pos,
        1536 - pos,
        "]}"
    );

    httpd_resp_set_type(
        r,
        "application/json"
    );

    esp_err_t respuesta =
        httpd_resp_sendstr(
            r,
            j
        );

    free(j);

    return respuesta;
}

static esp_err_t datos_csv(httpd_req_t *r)
{
    asegurar_csv();

    FILE *f =
        fopen(
            CSV_PATH,
            "rb"
        );

    if (!f) {
        return httpd_resp_send_err(
            r,
            HTTPD_404_NOT_FOUND,
            "Sin datos"
        );
    }

    httpd_resp_set_type(
        r,
        "text/csv"
    );

    httpd_resp_set_hdr(
        r,
        "Content-Disposition",
        "attachment; filename=\"detecciones_cores3.csv\""
    );

    char *b = static_cast<char *>(heap_caps_malloc(512, MALLOC_CAP_SPIRAM | MALLOC_CAP_8BIT)); if (b == nullptr) { fclose(f); return httpd_resp_send_err(r, HTTPD_500_INTERNAL_SERVER_ERROR, "Sin memoria"); }

    while (true) {
        size_t z =
            fread(
                b,
                1,
                512,
                f
            );

        if (!z) break;

        if (
            httpd_resp_send_chunk(
                r,
                b,
                z
            ) != ESP_OK
        ) {
            fclose(f);
            free(b);
            return ESP_FAIL;
        }
    }

    fclose(f);
    free(b);

    return httpd_resp_send_chunk(
        r,
        nullptr,
        0
    );
}

static bool iniciar_http()
{
    if (server != nullptr) return true;

    httpd_config_t c =
        HTTPD_DEFAULT_CONFIG();

    c.stack_size = 4096;
    c.max_open_sockets = 3;
    c.max_uri_handlers = 8;
    c.lru_purge_enable = true;
    c.send_wait_timeout = 20;
    c.recv_wait_timeout = 10;

    if (
        httpd_start(
            &server,
            &c
        ) != ESP_OK
    ) {
        return false;
    }

    httpd_uri_t a1 = {};
    a1.uri = "/";
    a1.method = HTTP_GET;
    a1.handler = root;

    httpd_uri_t a2 = {};
    a2.uri = "/capturar";
    a2.method = HTTP_POST;
    a2.handler = capt;

    httpd_uri_t a3 = {};
    a3.uri = "/foto";
    a3.method = HTTP_GET;
    a3.handler = foto;

    httpd_uri_t a4 = {};
    a4.uri = "/api";
    a4.method = HTTP_GET;
    a4.handler = api;

    httpd_uri_t a5 = {};
    a5.uri = "/datos.csv";
    a5.method = HTTP_GET;
    a5.handler = datos_csv;

    httpd_uri_t a6 = {};
    a6.uri = "/eventos.csv";
    a6.method = HTTP_GET;
    a6.handler = eventos_csv_pc;

    httpd_uri_t a7 = {};
    a7.uri = "/captura_historial";
    a7.method = HTTP_GET;
    a7.handler = captura_historial_pc;

    ESP_ERROR_CHECK(
        httpd_register_uri_handler(
            server,
            &a1
        )
    );

    ESP_ERROR_CHECK(
        httpd_register_uri_handler(
            server,
            &a2
        )
    );

    ESP_ERROR_CHECK(
        httpd_register_uri_handler(
            server,
            &a3
        )
    );

    ESP_ERROR_CHECK(
        httpd_register_uri_handler(
            server,
            &a4
        )
    );

    ESP_ERROR_CHECK(
        httpd_register_uri_handler(
            server,
            &a5
        )
    );

    ESP_ERROR_CHECK(
        httpd_register_uri_handler(
            server,
            &a6
        )
    );

    ESP_ERROR_CHECK(
        httpd_register_uri_handler(
            server,
            &a7
        )
    );

    ESP_LOGI(
        TAG,
        "Servidor HTTP listo"
    );

    return true;
}

bool web_panel_iniciar_wifi_temprano()
{
    if (wifi_temprano_iniciado) {
        return true;
    }

    ESP_LOGI(
        TAG,
        "WIFI STA: iniciando antes de YOLO"
    );

    if (!iniciar_sta()) {
        ESP_LOGE(
            TAG,
            "WIFI STA: fallo iniciar_sta()"
        );
        return false;
    }

    wifi_temprano_iniciado = true;

    ESP_LOGI(
        TAG,
        "WIFI STA: driver iniciado"
    );

    return true;
}

bool web_panel_iniciar_http_tarde()
{
    if (http_tarde_iniciado) {
        return true;
    }

    if (!wifi_temprano_iniciado) {
        if (!web_panel_iniciar_wifi_temprano()) {
            return false;
        }
    }

    ESP_LOGI(
        TAG,
        "HTTP TARDE: iniciando servidor"
    );

    if (!iniciar_http()) {
        ESP_LOGE(
            TAG,
            "HTTP TARDE: fallo iniciar_http()"
        );
        return false;
    }

    asegurar_csv();
    http_tarde_iniciado = true;
    panel_iniciado = true;

    ESP_LOGI(
        TAG,
        "HTTP TARDE: servidor listo http://192.168.4.1"
    );

    return true;
}

bool web_panel_iniciar()
{
    if (panel_iniciado) {
        return true;
    }

    if (!web_panel_iniciar_wifi_temprano()) {
        return false;
    }

    return web_panel_iniciar_http_tarde();
}

bool web_panel_consumir_captura()
{
    return captura.exchange(false);
}

void web_panel_actualizar_resultado(
    int h,
    int p,
    uint64_t t,
    float a,
    float b
)
{
    mh.store(h);
    pp.store(p);
    ms.store(t);
    ch.store((int)(a * 1000.0f));
    cp.store((int)(b * 1000.0f));

    guardar_csv(
        h,
        p,
        t,
        a,
        b
    );
}




bool web_panel_diagnostico_wifi()
{
    wifi_mode_t modo =
        WIFI_MODE_NULL;

    esp_err_t err =
        esp_wifi_get_mode(
            &modo
        );

    if (err != ESP_OK) {
        ESP_LOGE(
            TAG,
            "DIAG WIFI STA: get_mode fallo: %s",
            esp_err_to_name(err)
        );

        return false;
    }

    wifi_ap_record_t ap_info = {};

    esp_err_t conectado =
        esp_wifi_sta_get_ap_info(
            &ap_info
        );

    esp_netif_t *sta =
        esp_netif_get_handle_from_ifkey(
            "WIFI_STA_DEF"
        );

    esp_netif_ip_info_t ip = {};

    esp_err_t ip_err =
        sta != nullptr
            ? esp_netif_get_ip_info(
                  sta,
                  &ip
              )
            : ESP_FAIL;

    if (
        conectado == ESP_OK &&
        ip_err == ESP_OK &&
        ip.ip.addr != 0
    ) {
        ESP_LOGI(
            TAG,
            "DIAG WIFI STA: CONECTADO | SSID=%s | IP=" IPSTR " | RSSI=%d",
            reinterpret_cast<const char *>(
                ap_info.ssid
            ),
            IP2STR(
                &ip.ip
            ),
            ap_info.rssi
        );
    }
    else {
        ESP_LOGI(
            TAG,
            "DIAG WIFI STA: conectando... modo=%d wifi=%s ip=%s",
            static_cast<int>(
                modo
            ),
            esp_err_to_name(
                conectado
            ),
            esp_err_to_name(
                ip_err
            )
        );

        if (
            conectado ==
            ESP_ERR_WIFI_NOT_CONNECT
        ) {
            esp_err_t retry =
                esp_wifi_connect();

            ESP_LOGI(
                TAG,
                "WIFI STA RETRY: %s",
                esp_err_to_name(
                    retry
                )
            );
        }
    }

    return true;
}





































