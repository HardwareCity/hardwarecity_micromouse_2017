cmake_minimum_required(VERSION 2.8.4)
set(CMAKE_TOOLCHAIN_FILE ${CMAKE_SOURCE_DIR}/cmake/ArduinoToolchain.cmake)
set(PROJECT_NAME src)
project(${PROJECT_NAME})

print_board_list()

#### Uncomment below additional settings as needed.
# set(${CMAKE_PROJECT_NAME}_BOARD mega)
# set(${CMAKE_PROJECT_NAME}_BOARD nano)
set(${CMAKE_PROJECT_NAME}_PORT /dev/cu.wchusbserial1420)
# set(mega.build.mcu atmega2560)
# set(mega.upload.protocol wiring)
# set(mega.upload.speed 115200)

set(nano.upload.maximum_size 30720)
set(nano.upload.maximum_data_size 2048)
set(nano.upload.speed 57600)
set(nano.bootloader.low_fuses 0xFF)
set(nano.bootloader.high_fuses 0xDA)
set(nano.bootloader.extended_fuses 0x05)
set(nano.bootloader.file atmega/ATmegaBOOT_168_atmega328.hex)
set(nano.build.mcu atmega328p)

set(${CMAKE_PROJECT_NAME}_SKETCH src.ino)

generate_arduino_firmware(${CMAKE_PROJECT_NAME})
