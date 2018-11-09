# (1) Install Updated CMAKE
sudo apt remove cmake
#(password: citi)
cd cmake-3.12.4
./bootstrap
make -j4
sudo make install
