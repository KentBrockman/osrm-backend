# Install dependencies for building on 16.04 as per https://github.com/Project-OSRM/osrm-backend/wiki/Building-on-Ubuntu
sudo apt-get update
sudo apt install build-essential git cmake pkg-config \
libbz2-dev libstxxl-dev libstxxl1v5 libxml2-dev \
libzip-dev libboost-all-dev lua5.2 liblua5.2-dev libluabind-dev libtbb-dev -y
sudo update-alternatives --install /usr/bin/gcc gcc /usr/bin/gcc-6 20
sudo update-alternatives --install /usr/bin/g++ g++ /usr/bin/g++-6 20
sudo update-alternatives --install /usr/bin/cpp cpp /usr/bin/cpp-6 20

# Run install process as per https://github.com/Project-OSRM/osrm-backend/wiki/Building%20OSRM
# cd /vagrant
# mkdir -p build
# cd build
# cmake .. -DCMAKE_BUILD_TYPE=Release
# cmake --build .
# sudo cmake --build . --target install
