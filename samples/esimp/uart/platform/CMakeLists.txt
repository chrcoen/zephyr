cmake_minimum_required(VERSION 3.20.0)

set(CMAKE_C_COMPILER clang-11)
set(CMAKE_CXX_COMPILER clang++-11)
set(CMAKE_EXPORT_COMPILE_COMMANDS ON)
set(CMAKE_CXX_STANDARD 14)

project(esimp_sample)

set(CMAKE_BUILD_TYPE Debug)

set(ESIMP_PATH "${CMAKE_CURRENT_LIST_DIR}/../../../../../modules/esimp")
set(SYSTEMC_PATH "${CMAKE_CURRENT_LIST_DIR}/../../../../../modules/systemc")


add_library(esimp STATIC IMPORTED)
set_target_properties(esimp PROPERTIES
  IMPORTED_LOCATION "${ESIMP_PATH}/build/src/libesimp.a"
  INTERFACE_INCLUDE_DIRECTORIES "${ESIMP_PATH}/include"
)

include(${ESIMP_PATH}/cmake/systemc.cmake)

add_executable(platform
    src/main.cpp
)


target_link_libraries(platform esimp systemc pthread dl rt)

