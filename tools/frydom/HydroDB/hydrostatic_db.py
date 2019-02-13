#!/usr/bin/env python
#  -*- coding: utf-8 -*-
"""Module to create the hydrostatic database for frydom """

import numpy as np


class HydrostaticDB(object):

    def __init__(self):
        self._matrix = np.zeros((3, 3))

    @property
    def k33(self):
        return self._matrix[0, 0]

    @k33.setter
    def k33(self, value):
        self._matrix[0, 0] = value

    @property
    def k44(self):
        return self._matrix[1, 1]

    @k44.setter
    def k44(self, value):
        self._matrix[1, 1] = value

    @property
    def k55(self):
        return self._matrix[2, 2]

    @k55.setter
    def k55(self, value):
        self._matrix[2, 2] = value

    @property
    def k34(self):
        return self._matrix[0, 1]

    @k34.setter
    def k34(self, value):
        self._matrix[0, 1] = value
        self._matrix[1, 0] = value

    @property
    def k35(self):
        return self._matrix[0, 2]

    @k35.setter
    def k35(self, value):
        self._matrix[0, 2] = value
        self._matrix[2, 0] = value

    @property
    def k45(self):
        return self._matrix[1, 2]

    @k45.setter
    def k45(self, value):
        self._matrix[1, 2] = value
        self._matrix[2, 1] = value

    @property
    def diagonal(self):
        return np.array([self.k33, self.k44, self.k55])

    @diagonal.setter
    def diagonal(self, value):
        self.k33 = value[0]
        self.k44 = value[1]
        self.k55 = value[2]

    @property
    def non_diagonal(self):
        return np.array([self.k34, self.k35, self.k45])

    @non_diagonal.setter
    def non_diagonal(self, value):
        self.k34 = value[0]
        self.k35 = value[1]
        self.k45 = value[2]

    @property
    def matrix(self):
        res = np.zeros((6, 6))
        res[2:5, 2:5] = self._matrix
        return res

    @matrix.setter
    def matrix(self, value):
        if value.shape[0] == 3 and value.shape[1] == 3:
            self._matrix = value
        elif value.shape[0] == 6 and value.shape[1] == 6:
            self._matrix = value[2:5, 2:5]
        else:
            print("warning : shape matrix mismatch")

    @property
    def matrix33(self):
        return self._matrix


