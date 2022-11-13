# SPDX-License-Identifier: Apache-2.0

zephyr_library()

zephyr_library_compile_definitions(NO_POSIX_CHEATS)

set(INSTRUMENTATION_FLAGS "SHELL:-Xclang -load" "SHELL:-Xclang ${CMAKE_CURRENT_SOURCE_DIR}/../../../../modules/esimp/build/instr/libsiminstr.so")

# zephyr_compile_options(${INSTRUMENTATION_FLAGS})
zephyr_compile_options(-fexceptions)

zephyr_library_sources(
   esimp.cpp
   log.c
)

zephyr_library_include_directories(
   ${ZEPHYR_BASE}/kernel/include
   ${ZEPHYR_BASE}/arch/systemc/include
    )
