Install FRyDoM {#tutorial_install_frydom}
==========================

Full portability is not operational yet, the cmake was developed for Linux only.

## 1) Install a C++ compiler

Only the GNU C++ compiler for Linux-based platforms has been tested so far. To install it, you need the following packages

    apt-get install gcc
    apt-get install build-essential

<small>Other compilers could work as well, but they might require changes to the CMake scripts.</small>

## 2) Install [CMake](http://www.cmake.org/cmake/resources/software.html)

The free CMake utility is used to manage the building process. However CMake 3.11 or higher is required. This version is
already embedded in the CLion 2018.3. It will also need some related packages (libcurl)<br>

    apt-get install cmake
    apt-get install libcurl-dev
    
You can check [here](https://peshmerge.io/how-to-install-cmake-3-11-0-on-ubuntu-16-04/) how to install the CMake 3.11.0.

## 3) Install a GIT client

On Linux, there are several good [options](https://git-scm.com/download/gui/linux).
Or you can just install git independently, using :

    apt-get install git


## 4) Download the project by cloning the Git repository

Download FRyDoM by performing a **clone** of the Git repository in a directory on your machine. 

    git clone link-to-the-repo
    
## 5) Install the missing dependencies : Irrlicht, Eigen, BLAS, Boost, HDF5

Installing Irrlicht is detailed in the [Irrlicht module](@ref module_irrlicht_installation).
All others packages can be installed simply :

    apt-get install libeigen3-dev
    apt-get install libblas-dev
    apt-get install libhdf5-dev
    apt-get install libboost-all-dev


## 6) Run CMake

Create a build directory, and enter it 

    mkdir build
    cd build

run CMake

    cmake ..

## 7) Compile the project

Still inside your build directory, you then have to build the target you want to execute. Several targets are available,
as tutorials. Let's just build the demo_environment one.

    make demo_Environment
    
This will build every libraries needed, mangaged by CMake, from Chrono:Engine to Mathutils.

## 8) Run the demo

Go to the directory, where the binaries were built. Here the tutorials are built in 'tests/demos'. 
Then execute the demo-Environment. 

    cd tests/demos
    ./demo_Environment 