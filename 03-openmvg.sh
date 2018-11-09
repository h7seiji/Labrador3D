# (4) Build OpenMVG-short
cd openMVG-short
mkdir build
cd build
cmake . ../
make -j4
sudo make install
