# exit immediately on any failed step
set -xe

mkdir -p deps
cd deps

curl -OL https://gitlab.com/libeigen/eigen/-/archive/3.3.9/eigen-3.3.9.tar.gz
tar -zxf eigen-3.3.9.tar.gz

rm -rf libccd
git clone --depth 1 --branch v2.1 https://github.com/danfis/libccd.git

rm -rf octomap 
git clone --depth 1 --branch v1.9.8 https://github.com/OctoMap/octomap.git

rm -rf fcl
git clone --depth 1 --branch v0.7.0 https://github.com/ambi-robotics/fcl.git

# Install eigen
cmake -B build -S eigen-3.3.9
cmake --install build

# Build and install libccd
cd libccd
cmake .
make -j4
make install
cd ..

# Build and install octomap
cd octomap 
cmake . -D CMAKE_BUILD_TYPE=Release -D BUILD_OCTOVIS_SUBPROJECT=OFF -D BUILD_DYNAMICETD3D_SUBPROJECT=OFF
make -j4
make install
cd ..

# Build and install fcl
cd fcl
cmake .
make -j4
make install
cd ..

cd ..
