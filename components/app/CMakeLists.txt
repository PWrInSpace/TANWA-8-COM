idf_component_register(  SRC_DIRS "."
                         INCLUDE_DIRS "."
                         REQUIRES config can timers lora sd_card esp_now_sub proto data state_machine lora_commands)

target_compile_options(${COMPONENT_LIB} PRIVATE "-Wno-format" "-Wall" "-Werror")
