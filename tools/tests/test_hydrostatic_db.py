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
from HydroDB.hydrostatic_db import HydrostaticDB
import numpy as np
from math import *


def main():

    print("==========================================================\n"
          "Test : hydrostatic database\n"
          "==========================================================\n")

    database = HydrostaticDB()

    database.k33 = 1e5
    database.k44 = 1e6
    database.k55 = 1e6
    database.non_diagonal = np.array([0.1, 0.2, 0.3])

    print("Matrix66:")
    print(database.matrix)

    print("Matrix33:")
    print(database.matrix33)

    return 0


if __name__ == "__main__":
    main()
