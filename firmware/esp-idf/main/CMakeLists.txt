idf_component_register( SRCS            "main.c" "mqtt.c" "wifi.c"
                        INCLUDE_DIRS    "."
                        REQUIRES        "pzem004tv30"
                        REQUIRES        spi_flash nvs_flash mqtt)
