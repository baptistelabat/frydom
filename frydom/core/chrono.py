#!/usr/bin/env python
#  -*- coding: utf-8 -*-

__all__ = ['core', 'fea', 'irrlicht', 'postprocess']

import os
import sys


class ChronoImportError(Exception):
    pass


_dir_here = os.path.abspath(os.path.dirname(__file__))
_chrono_python_modules = os.path.join(_dir_here, 'chrono', 'build', 'bin')

sys.path.insert(0, _chrono_python_modules)

_err_msg = "%s is not available and must be compiled with python support"

try:
    import ChronoEngine_python_core as core
except ImportError:
    raise ChronoImportError(_err_msg % 'ChronoEngine_python_core')

try:
    import ChronoEngine_python_fea as fea
except ImportError:
    raise ChronoImportError(_err_msg % 'ChronoEngine_python_fea')

try:
    import ChronoEngine_python_irrlicht as irrlicht
except ImportError:
    raise ChronoImportError(_err_msg % 'ChronoEngine_python_irrlicht')

try:
    import ChronoEngine_python_postprocess as postprocess
except ImportError:
    raise ChronoImportError(_err_msg % 'ChronoEngine_python_postprocess')
