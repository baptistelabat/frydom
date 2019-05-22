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
"""
    Module to create a hydrodynamic database for FRyDoM.
"""

import os
from math import *
import numpy as np

from bem_reader_v2 import *
from pyHDB import *
from discretization_db_v2 import DiscretizationDB

class HDB5(object):

    """
        Class HDB5 for dealing with *.h5 files.
    """

    def __init__(self):

        """
            Constructor of the class HDB5.
        """

        # HDB.
        self._pyHDB = pyHDB()

        # Discretization parameters.
        self._discretization = DiscretizationDB()

        # Initialization parameter.
        self._is_initialized = False

        return

    @property
    def body(self):

        """This subroutine returns all the bodies.

        Returns
        -------
        BodyDB
        """

        return self._pyHDB.bodies

    @property
    def discretization(self):

        """This subroutine returns the parameters of the discretization.

        Returns
        -------
        DiscretizationDB
        """

        return self._discretization

    def nemoh_reader(self, input_directory='.', nb_faces_by_wavelength=None):

        """This function reads the *.cal file and stores the data.

        Parameters
        ----------
        input_directory : string, optional
            Path to directory of *.cal file.
        nb_faces_by_wavelength : float, optional
            Number of panels per wave length.
        """

        if not os.path.isabs(input_directory):
            input_directory = os.path.abspath(input_directory)

        # Verifying there is the Nemoh.cal file inside input_directory
        nemoh_cal_file = os.path.join(input_directory, 'Nemoh.cal')
        if not os.path.isfile(nemoh_cal_file):
            raise AssertionError('Folder %s seems not to be a Nemoh calculation folder as '
                                 'we did not find Nemoh.cal' % input_directory)

        print('========================')
        print('Reading Nemoh results...')
        print('========================')

        if nb_faces_by_wavelength is None:
            nb_faces_by_wavelength = 10

        # Reading *.cal.
        NemohReader(self._pyHDB,cal_file=nemoh_cal_file, test=True, nb_face_by_wave_length=nb_faces_by_wavelength)

        print('-------> Nemoh data successfully loaded from "%s"' % input_directory)
        print("")

    def _initialize(self):

        """This function updates and improve the hydrodynamic database (computation of RK and diffraction loads, impulse response functions, interpolation, etc.)."""

        # Computing Froude-Krylov loads.
        self._pyHDB.Eval_Froude_Krylov_loads()

        # Printing input data.
        self._discretization.initialize(self._pyHDB)

    def Symmetry_HDB(self):

        """This function symmetrizes the HDB."""

        # Updating the wave directions.
        self._pyHDB._initialize_wave_dir()















