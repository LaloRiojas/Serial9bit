cmake_minimum_required(VERSION 3.5.0)
project(Serial_9bit VERSION 0.1.0 LANGUAGES C)

add_executable(Serial_9bit SerialPortThreads/main.c SerialPortThreads/Serial.c SerialPortThreads/receive.c SerialPortThreads/send.c)
add_library(Serial_9bit_lib SerialPortThreads/Serial.c)

set(CPACK_PROJECT_NAME ${PROJECT_NAME})
set(CPACK_PROJECT_VERSION ${PROJECT_VERSION})
include(CPack)
