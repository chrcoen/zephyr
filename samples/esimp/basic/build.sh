#!/bin/bash

set -e

make -C ../modules/esimp/build/
make -C samples/esimp/basic/platform/build/

if [ -d "build_mcu0" ]; then
    west build -d build_mcu0
else
    west build -d build_mcu0 -b cortex_sim samples/esimp/basic/mcu0 -- -DCMAKE_EXPORT_COMPILE_COMMANDS=ON -DZEPHYR_TOOLCHAIN_VARIANT=llvm
fi

samples/esimp/basic/platform/build/platform build_mcu0/zephyr/libsimulation.so

