
set(CMAKE_SYSTEM_NAME Generic)
cmake_minimum_required(VERSION 2.8)

project(Transmitter)

set(VERSION_MAJOR "0")
set(VERSION_MINOR "0")
set(VERSION_PATCH "0")
set(VERSION ${VERSION_MAJOR}.${VERSION_MINOR}.${VERSION_PATCH})

#set(CMAKE_CXX_COMPILER avr-g++)
#set(CMAKE_CXX_FLAGS "-Wall -Os -mmcu=${MCU} -DF_CPU=${FCPU}")

add_subdirectory(Nrf24l01_driver)
add_subdirectory(src)
