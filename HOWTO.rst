Clone the FRyDoM repository
===========================

The FRyDoM project is currently hosted on githost at git@d-ice.githost.io:frydom/frydom.git

To clone the repository, just do::

    >$ git clone git@d-ice.githost.io:frydom/frydom.git



Install dependencies
====================

To build FRyDoM you will need libs and tools on your machine.

Begin with a

    >$ sudo apt-get update

    >$ sudo apt-get upgrade

Install HDF5 with:

    >$ sudo apt-get install libhdf5-100 libhdf5-cpp-100 libhdf5-dev

Install Irrlicht with:

    >$ sudo apt-get -y install build-essential xserver-xorg-dev x11proto-xf86vidmode-dev libxxf86vm-dev mesa-common-dev libgl1-mesa-dev libglu1-mesa-dev libxext-dev libxcursor-dev freeglut3-dev
 
    >$ sudo apt-get install libirrlicht-dev libirrlicht-doc libirrlicht1.8 libirrlicht1.8-dbg

To execute the build script, you will also need git, pip, python 2.7 and cmake

    >$ sudo apt-get install git

    >$ sudo apt-get install python-pip python-dev build-essential 

    >$ sudo apt-get install cmake


Building Thirdparty software
============================

Getting thirdparty repos 
------------------------

    >$ git submodule init
    >$ git submodule update
    >$ git submodule pull

These 3 commands will initialize and download last version of needed repos

The ``build_thirdparty.py`` script is available in the thirdparty directory. This script has to be run on the command line
and can be followed by the expected build type (Debug, Release...). To build FRyDoM's thirparty dependencies, just do::

    >$ cd thirdparty
    >$ ./build_thirdparty.py [BUILD_TYPE]

where BUILD_TYPE can take the values *Debug*, *Release*, *RelWithDebInfo* or *MinSizeRel*. Default is *Release* if no
command line argument is given.

This should automate the build for every of the thirdparty FRyDoM software.


Building FRyDoM
===============

Just like Chrono, FRyDoM uses cmake for its build management. At the root of the FRyDoM repository, you just have to do::

    >$ cmake CMakeLists.txt
    >$ make
