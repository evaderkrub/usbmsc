cmake_minimum_required(VERSION 3.13)
project(usbmsc_tests C)
set(CMAKE_C_STANDARD 11)
enable_testing()

add_executable(test_parse test_parse.c ${CMAKE_CURRENT_SOURCE_DIR}/../src/usb_parse.c)
target_include_directories(test_parse PRIVATE ${CMAKE_CURRENT_SOURCE_DIR}/../src)
add_test(NAME parse COMMAND test_parse)
