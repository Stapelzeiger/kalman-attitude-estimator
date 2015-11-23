#!/bin/sh

mkdir -p build
cd build tests
cmake ..
make && ./tests
