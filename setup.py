import sys, os
from pathlib import Path
from setuptools import Extension, setup
import numpy


# Load __version__
exec((Path(__file__).parent / 'fcl/version.py').read_text())

# Libraries needed during compilation of extension
libraries = ['fcl', 'octomap']

# Customize build options per platform
supported_platforms = 'darwin', 'linux', 'bsd', 'win32'
current_platform = sys.platform

if current_platform in supported_platforms:

    # Extra compile args needed per platform
    compile_args = []

    # Windows include and library search paths
    if current_platform == 'win32':
        # Run build script to download and build libraries.

        include_dirs = []
        library_dirs = []
        include_dir = r'C:\Program Files (x86)\{}\include'
        library_dir = r'C:\Program Files (x86)\{}\lib'
        deps = 'ccd', 'octomap', 'fcl'

        for dep in deps:
            include_dirs.append(include_dir.format(dep))
            library_dirs.append(library_dir.format(dep))

        # Add necessary Windows-specific dependencies
        libraries.extend(['octomath', 'ccd', 'vcruntime'])

        # Extra args for MSVC
        compile_args.extend(['/std:c++latest', '/MD', '/w'])

    # Unix include and library search paths
    else:
        include_dirs = [
            '/usr/include',
            '/usr/local/include',
            '/usr/include/eigen3'
        ]
        library_dirs = [
            '/usr/lib',
            '/usr/local/lib'
        ]

        # Extra args for CC
        compile_args.append('-std=c++11')

    # Optionally support additional search paths
    sep = os.path.pathsep
    include_dirs.extend(os.environ.get('CPATH', '').split(sep))
    include_dirs.extend(os.environ.get('LD_LIBRARY_PATH', '').split(sep))

    # Add Numpy include directories
    include_dirs.append(numpy.get_include())

else:
    raise NotImplementedError(current_platform)

setup(
    name='python-fcl',
    version=__version__,
    description='Python bindings for the Flexible Collision Library',
    long_description='Python bindings for the Flexible Collision Library',
    url='https://github.com/BerkeleyAutomation/python-fcl',
    author='Matthew Matl',
    author_email='mmatl@eecs.berkeley.edu',
    license = 'BSD',
    classifiers=[
        'Development Status :: 3 - Alpha',
        'License :: OSI Approved :: BSD License',
        'Operating System :: POSIX :: Linux',
        'Programming Language :: Python :: 2',
        'Programming Language :: Python :: 2.7',
        'Programming Language :: Python :: 3',
        'Programming Language :: Python :: 3.0',
        'Programming Language :: Python :: 3.1',
        'Programming Language :: Python :: 3.2',
        'Programming Language :: Python :: 3.3',
        'Programming Language :: Python :: 3.4',
        'Programming Language :: Python :: 3.5',
        'Programming Language :: Python :: 3.6',
    ],
    keywords='fcl collision distance',
    packages=['fcl'],
    setup_requires=['cython'],
    install_requires=['numpy', 'cython'],
    ext_modules=[Extension(
        'fcl.fcl',
        ['fcl/fcl.pyx'],
        include_dirs=include_dirs,
        library_dirs=library_dirs,
        libraries=libraries,
        language='c++',
        extra_compile_args=compile_args
    )]
)

# On Windows, DLL dependencies must be copied over to the installation directory

