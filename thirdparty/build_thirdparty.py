#!/usr/bin/env python
#  -*- coding: utf-8 -*-

import os
import sys
import shutil
import requests
import urllib
import zipfile
from subprocess import call
from distutils.spawn import find_executable

abs_path_root = os.getcwd()


# For parallel make
try:
    import multiprocessing
    nb_core = multiprocessing.cpu_count()
except ImportError:
    nb_core = 1


make = find_executable('make')
cmake = find_executable('cmake')
git = find_executable('git')
pip = find_executable('pip')


def build_mathutils(build_type):
    print("\n\n==============================================================================")
    print("Building MathUtils library (header only)")
    print("==============================================================================")

    os.chdir('MathUtils')

    # First building thirdparty from MathUtils
    os.chdir('thirdparty')
    from MathUtils.thirdparty.build_thirdparty import build as build_mathutils_3rd
    build_mathutils_3rd()

    os.chdir('..')

    print("""
     __  __       _   _     _   _ _   _ _       _     _ _                          
    |  \/  | __ _| |_| |__ | | | | |_(_) |___  | |   (_) |__  _ __ __ _ _ __ _   _ 
    | |\/| |/ _` | __| '_ \| | | | __| | / __| | |   | | '_ \| '__/ _` | '__| | | |
    | |  | | (_| | |_| | | | |_| | |_| | \__ \ | |___| | |_) | | | (_| | |  | |_| |
    |_|  |_|\__,_|\__|_| |_|\___/ \__|_|_|___/ |_____|_|_.__/|_|  \__,_|_|   \__, |
                                                                             |___/ 
    """)

    # Building MathUtils itself
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

    print("""
    __   __ _    __  __ _                           
    \ \ / // \  |  \/  | |          ___ _ __  _ __  
     \ V // _ \ | |\/| | |   _____ / __| '_ \| '_ \ 
      | |/ ___ \| |  | | |__|_____| (__| |_) | |_) |
      |_/_/   \_\_|  |_|_____|     \___| .__/| .__/ 
                                       |_|   |_|
    """)

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

    call([make, "-j", str(nb_core)])

    os.chdir('../..')



def build_chrono(build_type):
    print("\n\n==============================================================================")
    print("Building chrono library with some modules (shared)")
    print("==============================================================================")

    print("""
      ____ _                             _     _ _                          
     / ___| |__  _ __ ___  _ __   ___   | |   (_) |__  _ __ __ _ _ __ _   _ 
    | |   | '_ \| '__/ _ \| '_ \ / _ \  | |   | | '_ \| '__/ _` | '__| | | |
    | |___| | | | | | (_) | | | | (_) | | |___| | |_) | | | (_| | |  | |_| |
     \____|_| |_|_|  \___/|_| |_|\___/  |_____|_|_.__/|_|  \__,_|_|   \__, |
                                                                      |___/ 
    """)

    # New chrono_build directory
    try:
        os.mkdir("chrono_build")
    except OSError:
        pass

    # TODO: il faut regler les pb de chemin absolu dans chronoSettings.cmake...

    # irrlicht_dir = os.path.join(abs_path_root, "irrlicht-1.8.2")
    # irrlicht_lib = os.path.join(irrlicht_dir, "lib", "Linux", "libIrrlicht.a")  # FIXME: chemin non portable !!

    # Chrono Settings
    os.chdir('chrono_build')
    call([cmake, "-Wno-dev", "-C", "../cmakeCacheScripts/chronoSettings.cmake",
          #"-DCH_IRRLICHTDIR=%s" % irrlicht_dir,
          #"-DCH_IRRLICHTLIB=%s" % irrlicht_lib,
          "-DCMAKE_BUILD_TYPE=%s" % build_type,
          "../chrono"])

    call([make, "-j", str(nb_core)])
    os.chdir("..")


def clean():
    # Chrono
    print("Removing chrono build")
    shutil.rmtree("chrono_build")

    print("Removing MathUtils build")
    shutil.rmtree("MathUtils/build")

    print("Removing yaml-cpp build")
    shutil.rmtree("yaml-cpp/build")

    print("Removing libzmq build")
    shutil.rmtree("libzmq/build")

    print("Removing cppzmq build")
    shutil.rmtree("cppzmq/build")


def build(args):

    print("==============================================================================")
    print("Building thirdparty libraries for FRyDoM project")
    print("==============================================================================\n\n")

    if len(args) == 1:
        # By default, we build in Release mode
        build_type = "Debug"
    else:
        build_type = args[1]

        if (build_type == "clean"):
            clean()
            sys.exit(0)

        if build_type not in ("Debug", "Release", "RelWithDebInfo", "MinSizeRel"):
            raise NameError("Build type %s is not known. It has to be chosen among Debug, Release, RelWithDebInfo, MinSizeRel" % build_type)


    # Ensuring that thirdparty git submodules are up-to-date
    print("==============================================================================")
    print("Updating GIT SUBMODULES")
    print("==============================================================================")

    call([git, "submodule", "init"])
    call([git, "submodule", "update"])

    print("-- DONE.")

    # install_which()
    build_mathutils(build_type)
    build_yaml_cpp(build_type)
    build_chrono(build_type)


if __name__ == "__main__":

    build(sys.argv)
