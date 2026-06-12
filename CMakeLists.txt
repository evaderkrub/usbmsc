cmake_minimum_required(VERSION 3.13)

set(PICO_BOARD pico2 CACHE STRING "Board type")
set(PICO_PLATFORM rp2350-arm-s CACHE STRING "Platform")

include(pico_sdk_import.cmake)
project(usbmsc C CXX ASM)
set(CMAKE_C_STANDARD 11)
pico_sdk_init()

add_library(usbmsc_host
    src/usb_parse.c
    src/usb_hcd.c
    src/usb_core.c
    src/usb_msc.c
    src/usb_hub.c
)
target_include_directories(usbmsc_host PUBLIC src)
target_link_libraries(usbmsc_host PUBLIC pico_stdlib hardware_resets)

add_executable(bench apps/bench/main.c)
target_link_libraries(bench usbmsc_host)
pico_enable_stdio_uart(bench 1)
pico_enable_stdio_usb(bench 0)   # we own the USB controller — never link stdio_usb
pico_enable_stdio_rtt(bench 1)   # console over SWD debug probe (custom board, UART unwired)
pico_add_extra_outputs(bench)
