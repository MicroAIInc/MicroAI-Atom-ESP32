
set(COMPONENT_PRIV_REQUIRES "nvs_flash")
idf_component_register()
target_link_libraries(${COMPONENT_LIB} INTERFACE "-L ${CMAKE_CURRENT_SOURCE_DIR}")
target_link_libraries(${COMPONENT_LIB} INTERFACE AiEngine)
