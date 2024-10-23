#set(COMPONENT_SRCDIRS "src/aes src/hal src/lmic")
#set(COMPONENT_ADD_INCLUDEDIRS "include")
#set(REQUIRES "driver")
#register_component()

idf_component_register(SRC_DIRS "src/aes" "src/hal" "src/lmic"
INCLUDE_DIRS "include"
REQUIRES driver esp_timer)
