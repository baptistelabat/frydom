Install FRyDoM {#tutorial_install_frydom}
==========================

Full portability is not operational yet, the cmake was developed for Linux only.

## 1) Install a C++ compiler

Only the GNU C++ compiler for Linux-based platforms has been tested so far. To install it, you need the following package

    sudo apt-get install build-essential

<small>Other compilers could work as well, but they might require changes to the CMake scripts.</small>

## 2) Install CMake

The free CMake utility is used to manage the building process. However CMake 3.11 or higher is required. This version is
already embedded in the CLion 2018.3, but not in Ubuntu 18.04. It will also need some related packages (libcurl,libssl and zlib).

Start by installing libcurl, libssl and zlib:

    sudo apt-get install libssl-dev
    sudo apt-get install libcurl4-openssl-dev
    sudo apt-get install zlib1g-dev
    
then you can download the latest version of CMake, at the [official CMake webpage](https://cmake.org/download/).
You can extract the downloaded package in the /opt/ folder, in case you need to uninstall CMake later. 

    tar -xzvf cmake-$version.$build.tar.gz
    mv cmake-$version.$build /opt/
    
where version and build correspond to the version and build of the CMake you downloaded. (for me: it's cmake-3.13.3 so version = 3.13 and build = 3)

Once extracted, install CMake by running the following commands in the extracted folder:

    cd /opt/cmake-$version.$build
    sudo ./bootstrap --system-curl
    sudo make
    sudo make install
    
Check CMake version (you may need to close and reopen your terminal):

    cmake --version 
 
 For more details, check 
 
 * https://askubuntu.com/questions/355565/how-do-i-install-the-latest-version-of-cmake-from-the-command-line
 * https://github.com/ruslo/hunter/issues/328

## 3) Install a GIT client

On Linux, there are several good [options](https://git-scm.com/download/gui/linux).
Or you can just install git independently, using :

    sudo apt-get install git

Don't forget to add your ssh key to your gitlab account.
    
## 4) Download the project by cloning the Git repository

Download FRyDoM by performing a **clone** of the Git repository in a directory on your machine. 

    git clone link-to-the-repo

    
## 5) Install the missing dependencies : Irrlicht, Eigen, BLAS, Boost, HDF5

Installing Irrlicht is detailed in the [Irrlicht module](@ref module_irrlicht_installation).

All others packages can be installed simply :

    sudo apt-get install libeigen3-dev
    sudo apt-get install libblas-dev
    sudo apt-get install libhdf5-dev
    sudo apt-get install libboost-all-dev


## 6) Run CMake

Create a build directory, and enter it 
    
    cd path/to/frydom_ce/repository
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
