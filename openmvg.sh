# (4) Build OpenMVG-short

sudo apt-get install libpng-dev libjpeg-dev libtiff-dev

cd openMVG-short
mkdir build
cd build
cmake . ../
make -j4
sudo make install
