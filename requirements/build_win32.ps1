<#
***************************************************************************
Run using Administrator PowerShell prompt. WILL FAIL WITHOUT PROPER ACCESS
This is because the script places build artifacts in C:\Program Files (x86)
***************************************************************************

This script builds fcl and it's dependencies for python-fcl on Windows.

It downloads, builds, installs, and then deletes:
 * fcl
 * libccd
 * eigen
 * octomap
#>

# Ensure that paths are constant between runs
pushd $PSScriptRoot

# Create a directory that encapsulates all Git repos
mkdir fcl; cd fcl


#------------------------------------------------------------------------------
# Eigen
Write-Host "Building Eigen"
git clone https://gitlab.com/libeigen/eigen.git
cd eigen
mkdir build; cd build
cmake -DCMAKE_BUILD_TYPE=Release -G "Visual Studio 16 2019" -DBUILD_SHARED_LIBS=ON ..
cmake --build . --config Release --target install
Write-Host "Done"
cd ../..


# ------------------------------------------------------------------------------
# LibCCD
Write-Host "Building LibCCD"
git clone https://github.com/danfis/libccd
cd libccd
mkdir build; cd build
cmake -DCMAKE_BUILD_TYPE=Release -G "Visual Studio 16 2019" -DBUILD_SHARED_LIBS=ON ..
cmake --build . --config Release --target install
Write-Host "Done"
cd ../..

# FCL won't be able to find libccd unless it is named "ccd" exactly
Rename-Item "C:\Program Files (x86)\libccd" ccd


# ------------------------------------------------------------------------------
# Octomap
Write-Host "Building Octomap"
git clone https://github.com/OctoMap/octomap
cd octomap
mkdir build; cd build
cmake -DCMAKE_BUILD_TYPE=Release -G "Visual Studio 16 2019" -DBUILD_SHARED_LIBS=ON ..

# Build Octomap (won't install for some reason)
cmake --build . --config Release --target install

# Now rebuild, which installs correctly
cmake --build . --config Release --target install
Write-Host "Done"
cd ../../

# FCL won't be able to find octomap unless it is named "octomap" exactly
Rename-Item "C:\Program Files (x86)\octomap-distribution" octomap


# ------------------------------------------------------------------------------
# FCL
Write-Host "Building FCL"
git clone https://github.com/flexible-collision-library/fcl
cd fcl

# Checkout specific version 0.5.0 (only one compatible with python-fcl)
git checkout 7075caf32ddcd5825ff67303902e3db7664a407a
mkdir build; cd build

# Fiddle with build file to help fcl find LibCCD and Octomap
# Also tells compiler to dynamically link with C runtime (matches Cython)
$build_script_modification = @"
find_package(ccd QUIET)
find_package(octomap QUIET)
set(CMAKE_MSVC_RUNTIME_LIBRARY "MultiThreadedDLL$<$<CONFIG:Debug>:Debug>")
"@
$filePath = "../CMakeLists.txt"
$lineNumber = 11
$fileContent = Get-Content $filePath
$fileContent[$lineNumber-1] = $build_script_modification
$fileContent | Set-Content $filePath

# Now perform actual build
cmake -DCMAKE_BUILD_TYPE=Release -G "Visual Studio 16 2019" ..
cmake --build . --config Release --target install
Write-Host "Done"
cd ../../


# ------------------------------------------------------------------------------
# Python-FCL
cd ../

# Delete compilation directory since it is no longer needed
del -Force -Recurse fcl

cd ../

$fcl_dir = './fcl'
Write-Host "Copying dependent DLLs"
ls "C:/Program Files (x86)/octomap/bin/*.dll" | cp -destination $fcl_dir
cp "C:/Program Files (x86)/ccd/bin/ccd.dll" $fcl_dir

Start-Sleep -s 5

# Now build Cython extension
python setup-win32.py install
Write-Host "Successfully built python-fcl"


# Now delete all of the installed dependencies (after building python-fcl)
Write-Host "Removing build directories for Eigen, LibCCD, FCL, and Octomap"
rmdir -r 'C:\Program Files (x86)\Eigen3'
rmdir -r 'C:\Program Files (x86)\ccd'
rmdir -r 'C:\Program Files (x86)\fcl'
rmdir -r 'C:\Program Files (x86)\octomap'

Write-Host "All done!"

# Make sure to return the cwd to wherever it was before the build
popd
