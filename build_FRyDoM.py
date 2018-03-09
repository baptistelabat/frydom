#!/usr/bin/env python
#  -*- coding: utf-8 -*-

import sys
import os
import which

from subprocess import call


# For parallel make
try:
    import multiprocessing
    nb_core = multiprocessing.cpu_count()
except ImportError:
    nb_core = 1


try:
    make = which.which('make')
except which.WhichError:
    print ("cannot find make")
    # make = '/usr/bin/make'


try:
    cmake = which.which('cmake')
except which.WhichError:
    print "cannot find cmake"


if __name__ == '__main__':

    # # Building thirdparty first
    # from thirdparty.build_thirdparty import build as build_thirdparty
    # os.chdir("thirdparty")
    # build_thirdparty(sys.argv)
    # os.chdir("..")

    try:
        os.mkdir('build')
    except OSError:
        pass

    os.chdir('build')

    call([cmake, ".."])
    call([make, '-j', str(nb_core)])
