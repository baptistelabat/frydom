#!/usr/bin/env python
#  -*- coding: utf-8 -*-

import sys
from subprocess import call
sys.path.append("../thirdparty/which")
import which

ffmpeg = which.which('ffmpeg')

if __name__ == '__main__':

    if sys.argv[1] is not None:
        filename = str(sys.argv[1])
    else:
        filename = "frydom_video.mp4"

    call([ffmpeg,
          "-i", "screenshot%5d.bmp",
          "-c:v", "libx264",
          "-preset", "slow",
          "-crf", "21",
          "-r", "100",  # TODO: permettre de regler les fps ici...
          filename
          ])
