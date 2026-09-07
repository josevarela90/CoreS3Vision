idf_component_register(
    SRCS
        "main.cpp"
        "voz_martillo.c"
        "botones.cpp"
        "registro_flash.cpp"
        "web_panel.cpp"
    INCLUDE_DIRS "."
    REQUIRES esp_http_server esp_wifi esp_event esp_netif nvs_flash
)
