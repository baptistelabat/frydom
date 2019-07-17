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
"""Module to create the hydrostatic database for frydom """

import numpy as np

class HydrostaticDB(object):

    """
        Class for dealing with the hydrostatic parameters.
    """

    def __init__(self):

        """
        Constructor of the class HydrostaticDB.
        """

        self._matrix = np.zeros((3, 3))

    @property
    def k33(self):

        """This subroutine gives the coefficient K33 of the hydrostatic stiffness matrix.

        Returns
        -------
        float
            Coefficient K33 of the hydrostatic stiffness matrix.
        """

        return self._matrix[0, 0]

    @k33.setter
    def k33(self, value):

        """This subroutine sets the coefficient K33 of the hydrostatic stiffness matrix.

        Parameter
        ----------
        float : value
            Coefficient K33 of the hydrostatic stiffness matrix.
        """

        self._matrix[0, 0] = value

    @property
    def k44(self):

        """This subroutine gives the coefficient K44 of the hydrostatic stiffness matrix.

        Returns
        -------
        float
            Coefficient K44 of the hydrostatic stiffness matrix.
        """

        return self._matrix[1, 1]

    @k44.setter
    def k44(self, value):

        """This subroutine sets the coefficient K44 of the hydrostatic stiffness matrix.

        Parameter
        ----------
        float : value
            Coefficient K44 of the hydrostatic stiffness matrix.
        """

        self._matrix[1, 1] = value

    @property
    def k55(self):

        """This subroutine gives the coefficient K55 of the hydrostatic stiffness matrix.

        Returns
        -------
        float
            Coefficient K55 of the hydrostatic stiffness matrix.
        """

        return self._matrix[2, 2]

    @k55.setter
    def k55(self, value):

        """This subroutine sets the coefficient K55 of the hydrostatic stiffness matrix.

        Parameter
        ----------
        float : value
            Coefficient K55 of the hydrostatic stiffness matrix.
        """

        self._matrix[2, 2] = value

    @property
    def k34(self):

        """This subroutine gives the coefficient K34 of the hydrostatic stiffness matrix.

        Returns
        -------
        float
            Coefficient K34 of the hydrostatic stiffness matrix.
        """

        return self._matrix[0, 1]

    @k34.setter
    def k34(self, value):

        """This subroutine sets the coefficients K34 and K43 of the hydrostatic stiffness matrix.

        Parameter
        ----------
        float : value
            Coefficient K34 of the hydrostatic stiffness matrix.
        """

        self._matrix[0, 1] = value
        self._matrix[1, 0] = value

    @property
    def k35(self):

        """This subroutine gives the coefficient K35 of the hydrostatic stiffness matrix.

        Returns
        -------
        float
            Coefficient K35 of the hydrostatic stiffness matrix.
        """

        return self._matrix[0, 2]

    @k35.setter
    def k35(self, value):

        """This subroutine sets the coefficients K35 and K53 of the hydrostatic stiffness matrix.

        Parameter
        ----------
        float : value
            Coefficient K35 of the hydrostatic stiffness matrix.
        """

        self._matrix[0, 2] = value
        self._matrix[2, 0] = value

    @property
    def k45(self):

        """This subroutine gives the coefficient K45 of the hydrostatic stiffness matrix.

        Returns
        -------
        float
            Coefficient K45 of the hydrostatic stiffness matrix.
        """

        return self._matrix[1, 2]

    @k45.setter
    def k45(self, value):

        """This subroutine sets the coefficients K45 and K54 of the hydrostatic stiffness matrix.

        Parameter
        ----------
        float : value
            Coefficient K45 of the hydrostatic stiffness matrix.
        """

        self._matrix[1, 2] = value
        self._matrix[2, 1] = value

    @property
    def diagonal(self):

        """This subroutine gives the diagonal of the hydrostatic stiffness matrix.

        Returns
        -------
        Array of floats
            Diagonal of the hydrostatic stiffness matrix.
        """

        return np.array([self.k33, self.k44, self.k55])

    @diagonal.setter
    def diagonal(self, value):

        """This subroutine sets the diagonal of the hydrostatic stiffness matrix.

        Parameter
        ----------
        Array of floats : value
            Diagonal of the hydrostatic stiffness matrix.
        """

        self.k33 = value[0]
        self.k44 = value[1]
        self.k55 = value[2]

    @property
    def non_diagonal(self):

        """This subroutine gives the off-diagonal coefficients of the hydrostatic stiffness matrix.

        Returns
        -------
        Array of floats
            Off-diagonal coefficients of the hydrostatic stiffness matrix.
        """

        return np.array([self.k34, self.k35, self.k45])

    @non_diagonal.setter
    def non_diagonal(self, value):

        """This subroutine sets the off-diagonal coefficients of the hydrostatic stiffness matrix.

        Parameter
        ----------
        Array of floats : value
            Off-diagonal coefficients of the hydrostatic stiffness matrix.
        """

        self.k34 = value[0]
        self.k35 = value[1]
        self.k45 = value[2]

    @property
    def matrix(self):

        """This subroutine gives the hydrostatic stiffness matrix.

        Returns
        -------
        Array of floats
            Hydrostatique stiffness matrix.
        """

        res = np.zeros((6, 6))
        res[2:5, 2:5] = self._matrix
        return res

    @matrix.setter
    def matrix(self, value):

        """This subroutine sets the hydrostatic stiffness matrix.

        Parameter
        ----------
        Array of floats : value
            Hydrostatic stiffness matrix.
        """

        if value.shape[0] == 3 and value.shape[1] == 3:
            self._matrix = value
        elif value.shape[0] == 6 and value.shape[1] == 6:
            self._matrix = value[2:5, 2:5]
        else:
            print("warning : shape matrix mismatch")

    @property
    def matrix33(self):

        """This subroutine gives the reduced hydrostatic stiffness matrix.

        Returns
        -------
        Array of floats
            Reduced hydrostatique stiffness matrix.
        """

        return self._matrix


