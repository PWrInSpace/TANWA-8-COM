idf_component_register(
    SRCS "esp_now_wrapper.c" "ens_logic.c" "ens_config.c" "."
    INCLUDE_DIRS "."
    REQUIRES esp_wifi nvs_flash esp_timer
)