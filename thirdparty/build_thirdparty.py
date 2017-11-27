#!/usr/bin/env python
#  -*- coding: utf-8 -*-

import os
import sys
import shutil
sys.path.append("./which")
import which
from subprocess import call

abs_path_root = os.getcwd()


# For parallel make
try:
    import multiprocessing
    nb_core = multiprocessing.cpu_count()
except ImportError:
    nb_core = 1


try:
    make = which.which('make')
except which.WhichError:
    print "cannot find make"
    # make = '/usr/bin/make'

try:
    cmake = which.which('cmake')
except which.WhichError:
    print "cannot find cmake"
    cmake = '/home/frongere/mysofts/cmake-3.8.2-Linux-x86_64/bin/cmake'

try:
    git = which.which('git')
except which.WhichError:
    print "cannot find git"
    # cmake = '/usr/bin/git'
# try:
#     swig = which.which('swig')
# except which.WhichError:
#     print "cannot find swig"
#     swig = '/home/frongere/mysofts/miniconda2/bin/swig'

# cmake = which.which('cmake')
# swig = which.which('swig')

def build_eigen(build_type):
    print("\n\n==============================================================================")
    print("Building Eigen library (header only)")
    print("==============================================================================")
    try:
        os.mkdir('eigen_build')
    except OSError:
        pass

    os.chdir('eigen_build')
    call([cmake,
          '../eigen'
          ])
    # call([make])


    os.chdir('..')

def build_MathUtils(build_type):
    print("\n\n==============================================================================")
    print("Building MathUtils library (header only)")
    print("==============================================================================")
    os.chdir('MathUtils')

    try:
        os.mkdir('build')
    except OSError:
        pass
    os.chdir('build')

    call([cmake, '..'])
    call([make])





    os.chdir('../..')


def build_yaml_cpp(build_type):
    print("\n\n==============================================================================")
    print("Building YAML-CPP library (shared)")
    print("==============================================================================")
    os.chdir('yaml-cpp')

    try:
        os.mkdir('build')
    except OSError:
        pass

    os.chdir('build')
    call([cmake,
          "-DBUILD_SHARED_LIBS=ON",
          "-DCMAKE_BUILD_TYPE=%s" % build_type,
          ".."])
    # call([make, 'clean']) # TODO: retirer
    call([make, "-j", str(nb_core)])

    os.chdir('../..')


def build_irrlicht(build_type):

    print("\n\n==============================================================================")
    print("Building Irrlicht 1.8.2 library (static)")
    print("==============================================================================")
    os.chdir('irrlicht-1.8.2/source/Irrlicht')
    call([make, "-j", str(nb_core)])
    os.chdir('../../..')



def build_chrono(build_type):
    print("\n\n==============================================================================")
    print("Building chrono library with some modules (shared)")
    print("==============================================================================")

    # New chrono_build directory
    try:
        os.mkdir("chrono_build")
    except OSError:
        pass

    # TODO: il faut regler les pb de chemin absolu dans chronoSettings.cmake...

    irrlicht_dir = os.path.join(abs_path_root, "irrlicht-1.8.2")
    irrlicht_lib = os.path.join(irrlicht_dir, "lib", "Linux", "libIrrlicht.a")  # FIXME: chemin non portable !!

    # Chrono Settings
    os.chdir('chrono_build')
    call([cmake, "-Wno-dev", "-C", "../cmakeCacheScripts/chronoSettings.cmake",
          "-DCH_IRRLICHTDIR=%s" % irrlicht_dir,
          "-DCH_IRRLICHTLIB=%s" % irrlicht_lib,
          "-DCMAKE_BUILD_TYPE=%s" % build_type,
          "../chrono"])
    call([make, "-j", str(nb_core)])
    os.chdir("..")


def install_which():
    pip = which.which('pip')

    os.chdir('which')
    call([pip, "install", "-e", "."])
    os.chdir('..')


if __name__ == "__main__":

    print("==============================================================================")
    print("Building thirdparty libraries for FRyDoM project")
    print("==============================================================================\n\n")

    if len(sys.argv) == 1:
        # By default, we build in Release mode
        build_type = "Release"
    else:
        build_type = sys.argv[1]
        if build_type not in ("Debug", "Release", "RelWithDebInfo", "MinSizeRel"):
            raise NameError("Build type %s is not known. It has to be chosen among Debug, Release, RelWithDebInfo, MinSizeRel" % build_type)


    # Ensuring that thirdparty git submodules are up-to-date
    print("==============================================================================")
    print("Updating GIT SUBMODULES")
    print("==============================================================================")

    call([git, "submodule", "init"])
    call([git, "submodule", "update"])


    install_which()
    build_eigen(build_type)
    build_MathUtils(build_type)
    build_yaml_cpp(build_type)
    build_irrlicht(build_type)
    build_chrono(build_type)
