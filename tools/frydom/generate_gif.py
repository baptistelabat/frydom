#!/usr/bin/env python
#  -*- coding: utf-8 -*-
# ==========================================================================
# FRyDoM - frydom-ce.org
#
# Copyright (c) Ecole Centrale de Nantes (LHEEA lab.) and D-ICE Engineering.
# All rights reserved.
#
# Use of this source code is governed by a GPLv3 license that can be found
# in the LICENSE file of FRyDoM.
#
# ==========================================================================

import sys
from subprocess import call
from distutils.spawn import find_executable

ffmpeg = find_executable('ffmpeg')

if __name__ == '__main__':

    if sys.argv[1] is not None:
        filename = str(sys.argv[1])
    else:
        filename = "frydom_video.gif"

    call([ffmpeg,
          "-framerate", "40",
          "-pattern_type", "glob",
          "-i", "*.bmp",
          # "-r", "100", # 4 for lowRes # TODO: permettre de regler les fps ici...
          # "-vf", "scale=512:-1", # 256 for lowRes
          "-r", "4",  # 4 for lowRes # TODO: permettre de regler les fps ici...
          "-vf", "scale=256:-1",  # 256 for lowRes
          filename
          ])
