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


#------------------------------------------------------------------------------
# Eigen
Write-Host "Building Eigen"
$eigen_ver = "3.3.9"
Invoke-WebRequest -Uri https://gitlab.com/libeigen/eigen/-/archive/$eigen_ver/eigen-$eigen_ver.tar.gz -Outfile eigen-$eigen_ver.tar.gz
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
git clone --depth 1 --branch v2.1 https://github.com/danfis/libccd
Set-Location libccd

cmake -B build `
    -D CMAKE_BUILD_TYPE=Release `
    -G $generator `
    -D BUILD_SHARED_LIBS=ON `
    -D CMAKE_INSTALL_PREFIX=$install_dir
cmake --build build --config Release --target install

Set-Location ..


# ------------------------------------------------------------------------------
# Octomap
Write-Host "Building Octomap"
git clone --depth 1 --branch v1.9.8 https://github.com/OctoMap/octomap
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
git clone --depth 1 --branch v0.7.0 https://github.com/ambi-robotics/fcl.git
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
