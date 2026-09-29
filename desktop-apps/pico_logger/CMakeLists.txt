cmake_minimum_required(VERSION 3.13)

include(pico_sdk_import.cmake)

project(rhd_logger C CXX ASM)
set(CMAKE_C_STANDARD 11)
set(CMAKE_CXX_STANDARD 17)

pico_sdk_init()

add_executable(rhd_logger 
  src/main.cpp
)

pico_generate_pio_header(rhd_logger ${CMAKE_CURRENT_LIST_DIR}/src/ddr_spi.pio)

target_include_directories(rhd_logger 
  PRIVATE ${CMAKE_CURRENT_LIST_DIR}/include
)

# add_subdirectory(external/pico_fatfs)

pico_enable_stdio_usb(rhd_logger 1)
pico_enable_stdio_uart(rhd_logger 0)

pico_add_extra_outputs(rhd_logger)

target_link_libraries(rhd_logger
  pico_stdlib 
  pico_lwip_http
  pico_status_led
  pico_cyw43_arch_lwip_threadsafe_background
  # pico_fatfs
)
