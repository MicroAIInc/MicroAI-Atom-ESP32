idf_component_register(SRCS "esp32ai.c" "sensors.c" "mqtt.c"
                    INCLUDE_DIRS "." "../components/AiEngine")

set(COMPONENT_PRIV_REQUIRES "AiEngine")