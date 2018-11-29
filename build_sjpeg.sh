#!/bin/bash

sudo apt-get install freeglut3-dev

cd sjpeg
mkdir build
cd build
cmake . ..
make -j4

# make a copy inside appBoard folder
cp sjpeg ../../appBoard/bin/sjpeg

# using
# ./sjpeg image.jpg -o output.jpg -r 50
