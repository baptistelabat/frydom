Clone the FRyDoM repository
===========================

Begin with a

    >$ sudo apt-get update

    >$ sudo apt-get upgrade


The FRyDoM project is currently hosted on githost at https://d-ice.githost.io/Private_Sources/dice-frydom.git
So you'll need git to clone the repo :

    >$ sudo apt-get install git

To execute the build script, you will also need pip, python 2.7

    >$ sudo apt-get install python-pip python-dev build-essential

To clone the repository, just do::

    >$ git clone git@d-ice.githost.io:frydom/frydom.git


Install dependencies
====================

To build FRyDoM you will need libs and tools on your machine.

Install Boost submodules with :

    >$ sudo apt-get install libboost-filesystem1.62-dev (version may depends of your distro)


Install HDF5 with:

    >$ sudo apt-get install libhdf5-10 libhdf5-cpp-11 libhdf5-dev (version may depends of your distro)


Install Irrlicht with:

    >$ sudo apt-get -y install build-essential xserver-xorg-dev x11proto-xf86vidmode-dev libxxf86vm-dev mesa-common-dev libgl1-mesa-dev libglu1-mesa-dev libxext-dev libxcursor-dev freeglut3-dev

    >$ sudo apt-get install libirrlicht-dev libirrlicht-doc libirrlicht1.8 libirrlicht1.8-dbg


To run matplotlib and therefore MathUtils lib you need numpy

    >$ sudo apt-get install python-numpy

	or pip install numpy with your favorite virtual environment manager

To run Logger script you need both pyzmq, h5py and python protobuf libs

    >$ pip install protobuf pyzmq h5py

To correctly run protobuf and install all its dependancies run:

    >$ sudo apt-get install autoconf automake libtool curl make g++ unzip

Finally, you will need cmake

    >$ sudo apt-get install cmake


Building Thirdparty software
============================

Getting thirdparty repos
------------------------
    >$ cd dice-frydom
    >$ git submodule init
    >$ git submodule update

These 2 commands will initialize and download last version of needed repos

The ``build_thirdparty.py`` script is available in the thirdparty directory. This script has to be run on the command line
and can be followed by the expected build type (Debug, Release...). To build FRyDoM's thirparty dependencies, just do

    >$ cd thirdparty
    >$ ./build_thirdparty.py [BUILD_TYPE]

where BUILD_TYPE can take the values *Debug*, *Release*, *RelWithDebInfo* or *MinSizeRel*. Default is *Release* if no
command line argument is given.

This should automate the build for every of the thirdparty FRyDoM software.

Building FRyDoM
===============

Just like Chrono, FRyDoM uses cmake for its build management. At the root of the FRyDoM repository, you just have to do
    >$ mkdir build
    >$ cd build
    >$ cmake ..
    >$ make -j4
    >$ cd build
    >$ cmake ..
    >$ make
