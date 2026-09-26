idf_component_register(
    SRCS "app_main.cpp" "capture.cpp" "sd_logger.cpp" "report.cpp"
    INCLUDE_DIRS "."
    REQUIRES dsp esp_driver_i2s esp_driver_gpio esp_driver_sdspi fatfs sdmmc esp_timer esp_ringbuf
)
target_compile_options(${COMPONENT_LIB} PRIVATE -Wall -Wextra)
