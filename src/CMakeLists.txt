
set(CMAKE_SYSTEM_NAME Generic)
cmake_minimum_required(VERSION 2.8)

project(Transmitter)
set(MCU atmega8)
set(FCPU 7372800UL)

set(SOURCES
    main.cpp
    )

include_directories(
    ../Nrf24l01_driver
    ../Uart_driver
    ../Console
    ../OneWire_driver
    ../Gpio_driver
    ../Ds18b20
    ../External/include
    )

set(CMAKE_CXX_COMPILER avr-g++)
set(CMAKE_CXX_FLAGS "-std=c++0x -Wall -flto -Os -mmcu=${MCU} -DF_CPU=${FCPU}")
set(FLASH_FLAGS "-R .eeprom -R .fuse -R .lock -R .signature")

add_executable(${PROJECT_NAME}.elf ${SOURCES})

target_link_libraries(
    ${PROJECT_NAME}.elf
    Nrf24l01_driver
    Uart_driver
    Console
    Gpio_driver
    OneWire_driver
    Ds18b20
    )

add_custom_command(TARGET ${PROJECT_NAME}.elf
    POST_BUILD
    COMMAND avr-objcopy ${FLASH_FLAGS} -O ihex ${PROJECT_NAME}.elf ${PROJECT_NAME}.hex
    COMMAND echo "Generated hex file"
   )

#add_custom_target(gen_hex ALL
#    DEPENDS ${PROJECT_NAME}.elf
#    COMMAND avr-objcopy ${FLASH_FLAGS} -O ihex ${PROJECT_NAME}.elf ${PROJECT_NAME}.hex
#    )

set_directory_properties(PROPERTIES ADDITIONAL_MAKE_CLEAN_FILES ${PROJECT_NAME}.hex)
