cmake_minimum_required(VERSION 3.5)

project(XENSIV_PASCO2_CONTINUOUS LANGUAGES C)

file(GLOB SRCS ./*.c ../../corelib/*.h ../../corelib/*.c ../../wrapper/*.h ../../wrapper/*.c ../../pal/*.h ../../pal/*.c)
add_executable(XENSIV_PASCO2_CONTINUOUS ${SRCS})

target_include_directories(XENSIV_PASCO2_CONTINUOUS PRIVATE ../../corelib ../../wrapper ../../pal)
