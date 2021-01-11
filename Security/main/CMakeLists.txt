idf_component_register(SRCS "esp32ai.c" "mqtt.c"
                    INCLUDE_DIRS "." "../components/AiEngine" "../components/SecurityChannels")

set(COMPONENT_PRIV_REQUIRES "AiEngine")
set(COMPONENT_PRIV_REQUIRES "SecurityChannels")