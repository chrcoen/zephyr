#!/bin/bash

set -e

make -C ../modules/esimp/build/
make -C samples/esimp/uart/platform/build/

if [ -d "build_mcu0" ]; then
    west build -d build_mcu0
else
    west build -d build_mcu0 -b cortex_sim samples/esimp/uart/mcu0 -- -DCMAKE_EXPORT_COMPILE_COMMANDS=ON -DZEPHYR_TOOLCHAIN_VARIANT=llvm
fi

if [ -d "build_mcu1" ]; then
    west build -d build_mcu1
else
    west build -d build_mcu1 -b cortex_sim samples/esimp/uart/mcu1 -- -DCMAKE_EXPORT_COMPILE_COMMANDS=ON -DZEPHYR_TOOLCHAIN_VARIANT=llvm
fi

samples/esimp/uart/platform/build/platform build_mcu0/zephyr/libsimulation.so build_mcu1/zephyr/libsimulation.so
