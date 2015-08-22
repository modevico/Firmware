#!/bin/bash
d=$PWD && \
mkdir -p $d/build_host && cd $d/build_host && cmake .. -DTARGET_OS=posix -DTARGET_BOARD=sitl -DTARGET_LABEL=simple && make && ctest -V && cpack -G ZIP
