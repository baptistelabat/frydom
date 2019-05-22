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
        self.Added_mass = np.zeros((6, 6 * nb_bodies, nw), dtype = np.float)

        # Damping matrices (6 dof so 6 rows x all the columns x all the frequencies).
        self.Damping = np.zeros((6, 6 * nb_bodies, nw), dtype=np.float)

        # Diffraction loads (6 dof so 6 rows x all the frequencies).
        self.Diffraction = np.zeros((6, nw, nbeta), dtype=np.complex)

        # Froude-Krylov loads (6 dof so 6 rows x all the frequencies).
        self.Froude_Krylov = np.zeros((6, nw, nbeta), dtype=np.complex)

        # Excitation loads (6 dof so 6 rows x all the frequencies).
        self.Excitation = np.zeros((6, nw, nbeta), dtype=np.complex)

        # Mesh.
        self.mesh = mesh

        # Force mask.
        self.Force_mask = np.zeros(6,dtype = np.int)

        # Motion mask.
        self.Motion_mask = np.zeros(6,dtype = np.int)

        # Product n*dS for the computation of the Froude-Krylov loads.
        self._nds = None

        # Point of computation of the moments.
        self.point = np.zeros((3,3),dtype = np.float)

        # Impulse response functions without forward speed.
        self._irf_db = None

        # Impulse response functions with forward speed.
        self._irf_ku_db = None

    def _compute_nds(self):
        """Computes the term n dS for each force mode of the body."""

        self._nds = np.zeros((6, self.mesh.nb_faces), dtype=np.float)

        areas = self.mesh.faces_areas
        normals = self.mesh.faces_normals
        centers = self.mesh.faces_centers

        for i in range(0,6):
            direction = np.zeros(3)
            if (i == 0 or i == 4):
                direction[0] = 1  # ex.
            elif (i == 1 or i == 5):
                direction[1] = 1  # ey.
            elif (i == 2 or i == 6):
                direction[2] = 1  # ez.
            if i <= 2: # Force.
                self._nds[i, :] = areas * np.einsum('ij, j -> i', normals, direction)
            else:  # Moment.
                am = centers - self.point[i-3,:]
                vel = np.cross(direction, am)
                self._nds[i, :] = areas * (normals * vel).sum(axis=1)

    def get_nds(self, iforce):
        """Get the array of ndS . direction vector for the specified force mode.

        Parameters
        ----------
        iforce : int
            Force mode index.

        Returns
        -------
        np.ndarray
            (n_faces) Array of ndS quantities.
        """
        if self._nds is None:
            self._compute_nds()

        return self._nds[iforce, :]