#!/bin/bash

cd sjpeg
mkdir build
cd build
cmake . ..
make -j4

# make a copy inside appBoard folder
#cp sjpeg ../../appBoard/sjpeg

# using
# ./sjpeg image.jpg -o output.jpg -r 50
