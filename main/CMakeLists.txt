idf_component_register(
    SRCS
        "main.c"
        "can_rx.c"
        "can_tx.c"
        "espnow_link.c"
        "ui.c"
        "../components/waveshare_rgb_lcd_port.c"
    INCLUDE_DIRS
        "."
        "../components"
    EMBED_FILES
        "gauge_bg.bin")

idf_build_get_property(build_components BUILD_COMPONENTS)
if("lvgl" IN_LIST build_components)
    idf_component_get_property(lvgl_lib lvgl COMPONENT_LIB)
elseif("lvgl__lvgl" IN_LIST build_components)
    idf_component_get_property(lvgl_lib lvgl__lvgl COMPONENT_LIB)
endif()

if(lvgl_lib)
    target_compile_options(${lvgl_lib} PRIVATE -Wno-format)
endif()
