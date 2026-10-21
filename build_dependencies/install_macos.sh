# exit immediately on any failed step
set -xe
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
echo "7985975b787340124786f092b3a07d594b2e9cd53bbfe5f3d9b1daee7d55f56f  eigen-3.3.9.tar.gz" | shasum -a 256 -c -
tar -zxf eigen-3.3.9.tar.gz
mv eigen-3.3.9 eigen

clone_pinned https://github.com/danfis/libccd.git v2.1 7931e764a19ef6b21b443376c699bbc9c6d4fba8 libccd
clone_pinned https://github.com/OctoMap/octomap.git v1.9.8 d417c181868be79931ec94fd1a407c323e9f0fd3 octomap
clone_pinned https://github.com/ambi-robotics/fcl.git v0.7.0 06d38ed83d7bf33a0eeb7acb8e1d8edf13b12fb9 fcl

# Install eigen
$CMAKE -B build -S eigen
sudo $CMAKE --install build

# Build and install libccd
cd libccd
$CMAKE . -D ENABLE_DOUBLE_PRECISION=ON
make -j4
sudo make install
cd ..

# Build and install octomap
cd octomap
$CMAKE . -D CMAKE_BUILD_TYPE=Release -D BUILD_OCTOVIS_SUBPROJECT=OFF -D BUILD_DYNAMICETD3D_SUBPROJECT=OFF
make -j4
sudo make install
cd ..

# Build and install fcl
cd fcl
$CMAKE .
make -j4
sudo make install
cd ..

cd ..
