cmake_minimum_required(VERSION 3.20.0)

list(APPEND BOARD_ROOT ${CMAKE_CURRENT_SOURCE_DIR})

find_package(Zephyr REQUIRED HINTS $ENV{ZEPHYR_BASE})
project(volora)

file(GLOB CODEC2_SOURCES "lib/codec2/*.c")

file(GLOB APP_SOURCES "src/*.c")

target_sources(app PRIVATE ${APP_SOURCES} ${CODEC2_SOURCES})

target_include_directories(app PRIVATE lib/codec2)

target_compile_definitions(app PRIVATE
    CODEC2_MODE_EN_DEFAULT=0
    CODEC2_MODE_2400_EN=1
    __EMBEDDED__
    FDV_ARM_MATH
)
