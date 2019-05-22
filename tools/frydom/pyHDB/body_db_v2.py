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

"""Module to create a body database for frydom hydrodynamic database"""

import numpy as np

class BodyDB(object):

    """
        Class for writing the body data into the *.hdb5 file.
    """

    def __init__(self, i_body, nb_bodies, nw, nbeta, mesh):

        """
        Constructor of the class BodyDB.

        Parameters
        ----------
        i_body : int
            Index of the body.
        nb_bodies : int
            Number of bodies.
        nw : int
            Number of wave frequencies.
        nbeta : int
            Number of wave directions.
        mesh : Array of floats
            Mesh of the body.
        """

        # Index.
        self._i_body = None

        # Added mass matrices (6 dof so 6 rows x all the columns x all the frequencies).
        self.Added_mass = np.zeros((6,6*nb_bodies,nw), dtype = np.float)

        # Damping matrices (6 dof so 6 rows x all the columns x all the frequencies).
        self.Damping = np.zeros((6, 6 * nb_bodies, nw), dtype=np.float)

        # Diffraction loads (6 dof so 6 rows x all the frequencies).
        self.Diffraction = np.zeros((6, nw, nbeta), dtype=np.complex)

        # Froude-Krylov loads (6 dof so 6 rows x all the frequencies).
        self.FK = np.zeros((6, nw, nbeta), dtype=np.complex)

        # Excitation loads (6 dof so 6 rows x all the frequencies).
        self.Excitation = np.zeros((6, nw, nbeta), dtype=np.complex)

        # Mesh.
        self.mesh = mesh

        # Force mask.
        self.Force_mask = np.zeros(6,dtype = np.int)

        # Motion mask.
        self.Motion_mask = np.zeros(6,dtype = np.int)