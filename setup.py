import os
import sys

from Cython.Build import cythonize
from setuptools import Extension, setup

INSTALL_PREFIX_WIN = "deps\\install"


def is_nix_platform(platform):
    for prefix in ["darwin", "linux", "bsd"]:
        if prefix in sys.platform:
            return True
    return False


def get_include_dirs():
    if is_nix_platform(sys.platform):
        include_dirs = [
            "/usr/include",
            "/usr/local/include",
            "/usr/include/eigen3",
            "/usr/local/include/eigen3",
        ]

        if "CPATH" in os.environ:
            include_dirs += os.environ["CPATH"].split(":")

    elif sys.platform == "win32":
        include_dirs = [
            f"{INSTALL_PREFIX_WIN}\\include",
            f"{INSTALL_PREFIX_WIN}\\include\\eigen3",
        ]
    else:
        raise NotImplementedError(sys.platform)

    # get the numpy include path from numpy
    import numpy

    include_dirs.append(numpy.get_include())
    return include_dirs


def get_libraries_dir():
    if is_nix_platform(sys.platform):
        lib_dirs = ["/usr/lib", "/usr/local/lib"]

        if "LD_LIBRARY_PATH" in os.environ:
            lib_dirs += os.environ["LD_LIBRARY_PATH"].split(":")
        return lib_dirs
    if sys.platform == "win32":
        return [f"{INSTALL_PREFIX_WIN}\\lib"]

    raise NotImplementedError(sys.platform)


def get_libraries():
    libraries = ["fcl", "octomap"]
    if sys.platform == "win32":
        libraries.extend(["octomath", "ccd", "vcruntime"])
    return libraries


setup(
    ext_modules=cythonize(
        [
            Extension(
                "fcl.fcl",
                ["src/fcl/fcl.pyx"],
                include_dirs=get_include_dirs(),
                library_dirs=get_libraries_dir(),
                libraries=get_libraries(),
                language="c++",
                extra_compile_args=["-std=c++11"],
            )
        ],
    )
)
