idf_component_register(SRCS "led_esp32.c" "neopixel.c"
                       INCLUDE_DIRS "." "include"
                       REQUIRES crazyflie platform)