
set(CMAKE_SYSTEM_NAME Generic)
cmake_minimum_required(VERSION 2.8)

project(Transmitter)

set(VERSION_MAJOR "0")
set(VERSION_MINOR "0")
set(VERSION_PATCH "0")
set(VERSION ${VERSION_MAJOR}.${VERSION_MINOR}.${VERSION_PATCH})

message (STATUS "TARGET_PLATFORM = ${TARGET_PLATFORM}")
set (_TARGET_PLATFORM ${TARGET_PLATFORM})

add_subdirectory(Nrf24l01_driver)
add_subdirectory(Uart_driver)
add_subdirectory(Console)
add_subdirectory(OneWire_driver)
add_subdirectory(Gpio_driver)
add_subdirectory(Ds18b20)
add_subdirectory(src)
