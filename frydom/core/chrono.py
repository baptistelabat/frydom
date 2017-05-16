#!/usr/bin/env python
#  -*- coding: utf-8 -*-

__all__ = ['engine', 'fea', 'irrlicht', 'postprocessing']

import os
import sys


class ImportChronoError(Exception):
    pass


_dir_here = os.path.abspath(os.path.dirname(__file__))
_chrono_python_modules = os.path.join(_dir_here, 'chrono', 'build', 'bin')

sys.path.insert(0, _chrono_python_modules)

try:
    import ChronoEngine_python_core as engine
except ImportError:
    raise ImportChronoError('ChronoEngine core is not available and should be compiled with python support')

try:
    import ChronoEngine_python_fea as fea
except ImportError:
    raise ImportChronoError('ChronoEngine finite element module is not available and should be compiled with python '
                            'support')

try:
    import ChronoEngine_python_irrlicht as irrlicht
except ImportError:
    raise ImportChronoError('ChronoEngine irrlicht visualization support is not available and should be compiled with '
                            'python support')

try:
    import ChronoEngine_python_postprocess as postprocessing
except ImportError:
    raise ImportChronoError('ChronoEngine post-processing facilities is not available and should be compiled with '
                            'python support')
