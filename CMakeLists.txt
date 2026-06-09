cmake_minimum_required(VERSION 3.13)

include(pico_sdk_import.cmake)

project(test_project C CXX ASM)
set(CMAKE_C_STANDARD 11)
set(CMAKE_CXX_STANDARD 17)

pico_sdk_init()

add_executable(
  test_project src/main.cpp
)

target_include_directories(
  test_project PRIVATE ${CMAKE_CURRENT_LIST_DIR}/include
)

# add_subdirectory(external/pico_fatfs)

pico_enable_stdio_usb(test_project 1)
pico_enable_stdio_uart(test_project 0)

pico_add_extra_outputs(test_project)

target_link_libraries(
  test_project 
  pico_stdlib 
  pico_cyw43_arch_lwip_threadsafe_background
  # pico_fatfs
)
