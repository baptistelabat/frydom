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
"""Module to create the inertia matrix for the RAO computation."""

import numpy as np

class Inertia(object):

    """
        Class for dealing with the inertia parameters.
    """

    def __init__(self):

        """
        Constructor of the class Inertia.
        """

        self._matrix = np.zeros((3, 3))

    @property
    def mass(self):
        """This subroutine gives the mass.

        Returns
        -------
        float
            Mass.
        """

        return self._mass

    @mass.setter
    def mass(self, value):

        """This subroutine sets mass.

        Parameter
        ----------
        float : value
            Mass.
        """

        self._mass = value

    @property
    def I44(self):

        """This subroutine gives the coefficient I44 of the inertia matrix.

        Returns
        -------
        float
            Coefficient I44 of the inertia matrix.
        """

        return self._matrix[0, 0]

    @I44.setter
    def I44(self, value):

        """This subroutine sets the coefficient I44 of the inertia matrix.

        Parameter
        ----------
        float : value
            Coefficient I44 of the inertia matrix.
        """

        self._matrix[0, 0] = value

    @property
    def I55(self):

        """This subroutine gives the coefficient I55 of the inertia matrix.

        Returns
        -------
        float
            Coefficient I55 of the inertia matrix.
        """

        return self._matrix[1, 1]

    @I55.setter
    def I55(self, value):

        """This subroutine sets the coefficient I55 of the inertia matrix.

        Parameter
        ----------
        float : value
            Coefficient I55 of the inertia matrix.
        """

        self._matrix[1, 1] = value

    @property
    def I66(self):

        """This subroutine gives the coefficient I66 of the inertia matrix.

        Returns
        -------
        float
            Coefficient I66 of the inertia matrix.
        """

        return self._matrix[2, 2]

    @I66.setter
    def I66(self, value):

        """This subroutine sets the coefficient I66 of the inertia matrix.

        Parameter
        ----------
        float : value
            Coefficient I66 of the inertia matrix.
        """

        self._matrix[2, 2] = value

    @property
    def I45(self):

        """This subroutine gives the coefficient I45 of the inertia matrix.

        Returns
        -------
        float
            Coefficient I45 of the inertia matrix.
        """

        return self._matrix[0, 1]

    @I45.setter
    def I45(self, value):

        """This subroutine sets the coefficients I45 and I54 of the inertia matrix.

        Parameter
        ----------
        float : value
            Coefficient I45 of the inertia matrix.
        """

        self._matrix[0, 1] = value
        self._matrix[1, 0] = value

    @property
    def I46(self):

        """This subroutine gives the coefficient I46 of the inertia matrix.

        Returns
        -------
        float
            Coefficient I46 of the inertia matrix.
        """

        return self._matrix[0, 2]

    @I46.setter
    def I46(self, value):

        """This subroutine sets the coefficients I46 and I64 of the inertia matrix.

        Parameter
        ----------
        float : value
            Coefficient I46 of the inertia matrix.
        """

        self._matrix[0, 2] = value
        self._matrix[2, 0] = value

    @property
    def I56(self):

        """This subroutine gives the coefficient I56 of the inertia matrix.

        Returns
        -------
        float
            Coefficient I56 of the inertia matrix.
        """

        return self._matrix[1, 2]

    @I56.setter
    def I56(self, value):

        """This subroutine sets the coefficients I56 and I65 of the inertia matrix.

        Parameter
        ----------
        float : value
            Coefficient I56 of the inertia matrix.
        """

        self._matrix[1, 2] = value
        self._matrix[2, 1] = value

    @property
    def diagonal(self):

        """This subroutine gives the diagonal of the inertia matrix.

        Returns
        -------
        Array of floats
            Diagonal of the inertia matrix.
        """

        return np.array([self.I44, self.I55, self.I66])

    @diagonal.setter
    def diagonal(self, value):

        """This subroutine sets the diagonal of the inertia matrix.

        Parameter
        ----------
        Array of floats : value
            Diagonal of the inertia matrix.
        """

        self.I44 = value[0]
        self.I55 = value[1]
        self.I66 = value[2]

    @property
    def non_diagonal(self):

        """This subroutine gives the off-diagonal coefficients of the inertia matrix.

        Returns
        -------
        Array of floats
            Off-diagonal coefficients of the inertia matrix.
        """

        return np.array([self.I45, self.I46, self.I56])

    @non_diagonal.setter
    def non_diagonal(self, value):

        """This subroutine sets the off-diagonal coefficients of the inertia matrix.

        Parameter
        ----------
        Array of floats : value
            Off-diagonal coefficients of the inertia matrix.
        """

        self.I45 = value[0]
        self.I46 = value[1]
        self.I56 = value[2]

    @property
    def matrix(self):

        """This subroutine gives the mass matrix.

        Returns
        -------
        Array of floats
            Inertia matrix.
        """

        res = np.zeros((6, 6))
        res[3:6, 3:6] = self._matrix
        res[0,0] = self._mass
        res[1, 1] = self._mass
        res[2, 2] = self._mass
        return res

    @matrix.setter
    def matrix(self, value):

        """This subroutine sets the mass matrix.

        Parameter
        ----------
        Array of floats : value
            Inertia matrix.
        """

        if value.shape[0] == 6 and value.shape[1] == 6:
            self._matrix = value[3:6, 3:6]
            self._mass = value[0,0]
        else:
            print("warning : shape matrix mismatch")

    @property
    def matrix33(self):

        """This subroutine gives the reduced inertia matrix.

        Returns
        -------
        Array of floats
            Reduced inertia matrix.
        """

        return self._matrix


