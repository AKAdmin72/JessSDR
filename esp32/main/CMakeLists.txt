idf_component_register(SRCS "main.c" "lcd.c" "splash.c" "spectrum.c" "fpga.c" "input.c"
                       INCLUDE_DIRS "."
                       REQUIRES esp_lcd esp_driver_spi esp_driver_ledc esp_driver_i2s
                                esp_driver_gpio esp_timer nvs_flash esp_driver_uart)
