#!/bin/sh

mkdir -p lib
git clone git@github.com:Stapelzeiger/eigen-mirror.git lib/eigen

mkdir -p build
cd build tests
cmake ..
make && ./tests
