<#
Originally based on script written by Pebaz (https://github.com/Pebaz/python-fcl/blob/master/requirements/build_win32.ps1)
but with many modifications in order to use fcl 0.7.0 and install dependencies without admin rights.

This script builds fcl and it's dependencies for python-fcl on Windows.

It downloads, builds, installs:
 * fcl
 * libccd
 * eigen
 * octomap
#>

# Remember starting location for future usage
$base_dir = Get-Location

# Create a directory that encapsulates all dependencies
mkdir -p deps; Set-Location deps

# Build options
$generator = "Visual Studio 17 2022"

# All compiled dependencies will be installed in following folder
$install_dir = "$base_dir\deps\install"

# Clone a tag and refuse to build if it doesn't point at the expected commit
function Get-PinnedRepo($url, $tag, $sha, $dir) {
    git clone --depth 1 --branch $tag $url $dir
    $head = git -C $dir rev-parse HEAD
    if ($head -ne $sha) { throw "$dir is at $head, expected $sha" }
}


#------------------------------------------------------------------------------
# Eigen
Write-Host "Building Eigen"
$eigen_ver = "3.3.9"
Invoke-WebRequest -Uri https://gitlab.com/libeigen/eigen/-/archive/$eigen_ver/eigen-$eigen_ver.tar.gz -Outfile eigen-$eigen_ver.tar.gz
$eigen_sha256 = "7985975b787340124786f092b3a07d594b2e9cd53bbfe5f3d9b1daee7d55f56f"
if ((Get-FileHash "eigen-$eigen_ver.tar.gz" -Algorithm SHA256).Hash -ne $eigen_sha256) { throw "eigen-$eigen_ver.tar.gz checksum mismatch" }
tar -zxf "eigen-$eigen_ver.tar.gz"
Set-Location "eigen-$eigen_ver"

cmake -B build `
    -D CMAKE_BUILD_TYPE=Release `
    -G $generator `
    -D BUILD_SHARED_LIBS=ON `
    -D CMAKE_INSTALL_PREFIX=$install_dir
cmake --install build

Set-Location ..


# ------------------------------------------------------------------------------
# LibCCD
Write-Host "Building LibCCD"
Get-PinnedRepo https://github.com/danfis/libccd v2.1 7931e764a19ef6b21b443376c699bbc9c6d4fba8 libccd
Set-Location libccd

cmake -B build `
    -D CMAKE_BUILD_TYPE=Release `
    -G $generator `
    -D BUILD_SHARED_LIBS=ON `
    -D ENABLE_DOUBLE_PRECISION=ON `
    -D CMAKE_INSTALL_PREFIX=$install_dir
cmake --build build --config Release --target install

Set-Location ..


# ------------------------------------------------------------------------------
# Octomap
Write-Host "Building Octomap"
Get-PinnedRepo https://github.com/OctoMap/octomap v1.9.8 d417c181868be79931ec94fd1a407c323e9f0fd3 octomap
Set-Location octomap

cmake -B build `
    -D CMAKE_PREFIX_PATH=$install_dir `
    -D CMAKE_BUILD_TYPE=Release `
    -G $generator `
    -D BUILD_SHARED_LIBS=ON `
    -D CMAKE_INSTALL_PREFIX=$install_dir `
    -D BUILD_OCTOVIS_SUBPROJECT=OFF `
    -D BUILD_DYNAMICETD3D_SUBPROJECT=OFF
cmake --build build --config Release
cmake --build build --config Release --target install

Set-Location ..

# ------------------------------------------------------------------------------
# FCL
Write-Host "Building FCL"
Get-PinnedRepo https://github.com/ambi-robotics/fcl.git v0.7.0 06d38ed83d7bf33a0eeb7acb8e1d8edf13b12fb9 fcl
Set-Location fcl

cmake -B build `
    -D CMAKE_PREFIX_PATH=$install_dir `
    -D CMAKE_BUILD_TYPE=Release `
    -G $generator `
    -D CMAKE_INSTALL_PREFIX=$install_dir

cmake --build build --config Release --target install
Set-Location ..

# ------------------------------------------------------------------------------
# Python-FCL

Write-Host "Copying dependent DLLs"
Copy-Item $install_dir\bin\octomap.dll $base_dir\src\fcl
Copy-Item $install_dir\bin\octomath.dll $base_dir\src\fcl
Copy-Item $install_dir\bin\ccd.dll $base_dir\src\fcl

Set-Location $base_dir
Write-Host "All done!"
