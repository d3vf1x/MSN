idf_component_register(SRCS "main.c" "lora.c"
                       INCLUDE_DIRS "."
                       REQUIRES adc bme680 lmic-esp-idf board)

