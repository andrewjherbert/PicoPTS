
cmake_minimum_required(VERSION 3.13)

include(pico_sdk_import.cmake)

project(PicoPTS)
pico_sdk_init()

add_executable(picopts
    picopts.c
  )

target_link_libraries(picopts
  pico_stdlib
  pico_multicore)

pico_enable_stdio_usb(picopts 0)
pico_enable_stdio_uart(picopts 1)

pico_add_extra_outputs(picopts)

