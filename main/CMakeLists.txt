idf_component_register(SRCS "main.c"
                    INCLUDE_DIRS "."
                    EMBED_TXTFILES "index.html" "index.js"
                    REQUIRES esp_http_server esp_adc nvs_flash esp_wifi driver esp_netif freertos esp_event)

