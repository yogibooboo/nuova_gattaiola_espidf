# CMakeLists.txt per il componente principale in src/

# Registra il componente principale
idf_component_register(
    SRCS "main.cpp" "wifi.cpp" "comune.cpp" "console.cpp" "buffers.psram.cpp" "telnet_server.cpp" "time_sync.cpp" 
    "core1.cpp" "door.cpp" "http_server.cpp" "ota_handlers.cpp" "system_diag.cpp"
    INCLUDE_DIRS "../include"
    REQUIRES esp_http_server esp_wifi nvs_flash spiffs
)
