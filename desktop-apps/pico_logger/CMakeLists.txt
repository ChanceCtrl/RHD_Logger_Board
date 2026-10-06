cmake_minimum_required(VERSION 3.13)

include(pico_sdk_import.cmake)

project(rhd_logger C CXX ASM)
set(CMAKE_C_STANDARD 11)
set(CMAKE_CXX_STANDARD 17)

pico_sdk_init()

add_executable(${PROJECT_NAME}
  src/main.cpp
)

pico_generate_pio_header(${PROJECT_NAME} ${CMAKE_CURRENT_LIST_DIR}/src/ddr_spi.pio)

target_include_directories(${PROJECT_NAME} 
PRIVATE 
  ${CMAKE_CURRENT_LIST_DIR}/include
  ${CMAKE_CURRENT_LIST_DIR}/external/flatcc/include/
)

pico_enable_stdio_usb(${PROJECT_NAME} 1)
pico_enable_stdio_uart(${PROJECT_NAME} 0)

pico_add_extra_outputs(${PROJECT_NAME})

add_library(flatcc_runtime STATIC
  ${CMAKE_CURRENT_LIST_DIR}/external/flatcc/src/runtime/builder.c
  ${CMAKE_CURRENT_LIST_DIR}/external/flatcc/src/runtime/emitter.c
  ${CMAKE_CURRENT_LIST_DIR}/external/flatcc/src/runtime/refmap.c
)

target_include_directories(flatcc_runtime
  PUBLIC
    ${CMAKE_CURRENT_LIST_DIR}/external/flatcc/include
)

target_link_libraries(${PROJECT_NAME}
  pico_stdlib 
  flatcc_runtime
  pico_lwip_http
  pico_status_led
  pico_cyw43_arch_lwip_threadsafe_background
  # pico_fatfs
)
