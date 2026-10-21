# exit immediately on any failed step
set -xe

# get cmake from pip so we can version-lock
pip install cmake==3.31.6
# Set CMAKE variable to use the cmake in the same directory as python
export CMAKE="$(dirname $(which python))/cmake"

$CMAKE --version

# clone a tag and refuse to build if it doesn't point at the expected commit
clone_pinned() {
  rm -rf "$4"
  git clone --depth 1 --branch "$2" "$1" "$4"
  test "$(git -C "$4" rev-parse HEAD)" = "$3"
}

mkdir -p deps
cd deps

curl -fsSLO https://gitlab.com/libeigen/eigen/-/archive/3.3.9/eigen-3.3.9.tar.gz
echo "7985975b787340124786f092b3a07d594b2e9cd53bbfe5f3d9b1daee7d55f56f  eigen-3.3.9.tar.gz" | sha256sum -c -
tar -zxf eigen-3.3.9.tar.gz

clone_pinned https://github.com/danfis/libccd.git v2.1 7931e764a19ef6b21b443376c699bbc9c6d4fba8 libccd
clone_pinned https://github.com/OctoMap/octomap.git v1.9.8 d417c181868be79931ec94fd1a407c323e9f0fd3 octomap
clone_pinned https://github.com/ambi-robotics/fcl.git v0.7.0 06d38ed83d7bf33a0eeb7acb8e1d8edf13b12fb9 fcl

# Install eigen
$CMAKE -B build -S eigen-3.3.9 
$CMAKE --install build

# Build and install libccd
cd libccd
$CMAKE . -DENABLE_DOUBLE_PRECISION=ON
make -j4
make install
cd ..

# Build and install octomap
cd octomap
$CMAKE .  -DCMAKE_BUILD_TYPE=Release -DBUILD_OCTOVIS_SUBPROJECT=OFF -DBUILD_DYNAMICETD3D_SUBPROJECT=OFF -DCMAKE_CXX_STANDARD=17 -DCMAKE_CXX_STANDARD_REQUIRED=ON -DCMAKE_CXX_FLAGS="-std=c++17 -Wno-error=template-id-cdtor"
make -j4
make install
cd ..

# Build and install fcl
cd fcl
$CMAKE . 
make -j4
make install
cd ..

# Update library cache so the system can find the newly installed libraries
ldconfig
