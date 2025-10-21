# exit immediately on any failed step
set -xe

# get cmake from pip so we can version-lock
pip install cmake==3.31.6
# Set CMAKE variable to use the cmake in the same directory as python
export CMAKE="$(dirname $(which python))/cmake"

$CMAKE --version

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
$CMAKE -B build -S eigen-3.3.9
$CMAKE --install build

# Build and install libccd
cd libccd
$CMAKE . -D ENABLE_DOUBLE_PRECISION=ON
make -j4
make install
cd ..

# Build and install octomap
cd octomap
$CMAKE . -D CMAKE_BUILD_TYPE=Release -D BUILD_OCTOVIS_SUBPROJECT=OFF -D BUILD_DYNAMICETD3D_SUBPROJECT=OFF -D CMAKE_CXX_STANDARD=17
make -j4
make install
cd ..

# Build and install fcl
cd fcl
$CMAKE .
make -j4
make install
cd ..

cd ..
