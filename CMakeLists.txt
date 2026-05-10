# SPDX-License-Identifier: Apache-2.0

cmake_minimum_required(VERSION 3.20.0)
find_package(Zephyr REQUIRED HINTS $ENV{ZEPHYR_BASE})
project(blinky)

# Codec2 sources — exclude dump.c (file I/O, not needed on-target)
file(GLOB CODEC2_SOURCES "lib/codec2/*.c")
list(REMOVE_ITEM CODEC2_SOURCES "${CMAKE_CURRENT_SOURCE_DIR}/lib/codec2/dump.c")

target_sources(app PRIVATE src/main.c src/codec2_alloc.c ${CODEC2_SOURCES})

target_include_directories(app PRIVATE lib/codec2 src)

target_compile_definitions(app PRIVATE
    CODEC2_MODE_EN_DEFAULT=0
    CODEC2_MODE_2400_EN=1
    __EMBEDDED__
    FDV_ARM_MATH
)
