#!/bin/bash

rm -rf build

mkdir build
cd build
CC=clang CXX=clang++ cmake -DCMAKE_EXPORT_COMPILE_COMMANDS=1 \
                           -DCMAKE_VERBOSE_MAKEFILE=0 \
                           -DCMAKE_BUILD_TYPE=Release \
                           -DUSE_CCACHE=ON \
                           -DUSE_SANITIZER=OFF \
                           -DBUILD_WITH_MARCH_NATIVE=ON \
                           -DUSE_STACK_TRACE_LOGGER=OFF \
                           ..
cd ..

mv build/compile_commands.json .
