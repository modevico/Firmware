#!/bin/bash
d=$PWD && \
mkdir -p $d/build_arm && cd $d/build_arm && cmake -DCMAKE_TOOLCHAIN_FILE=../cmake/Toolchain-arm-none-eabi.cmake .. && make && ctest -V && cpack -G ZIP 
