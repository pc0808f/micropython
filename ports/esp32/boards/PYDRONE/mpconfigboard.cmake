include(boards/mpconfigboard_esp32s3_common.cmake)

set(MICROPY_PORT_DRONE y) #CAM
set(MICROPY_PORT_PICLIB y)
set(MICROPY_PORT_CAMLIB y) #CAM
set(MICROPY_PORT_WEB_STREAM y) #WEB stream

list(APPEND SDKCONFIG_DEFAULTS
    boards/sdkconfig.ble
    boards/sdkconfig.usb
    boards/PYDRONE/sdkconfig.board
    boards/sdkconfig.cam
)

set(MICROPY_PY_ESP32_ULP 0)
list(APPEND MICROPY_CPP_FLAGS_EXTRA
    -I/opt/esp/idf/components/ulp/ulp_fsm/include
    -I/opt/esp/idf/components/ulp/ulp_common/include
)
