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
"""Module to create the discretization database for frydom hydrodynamic database"""

from math import *
import numpy as np

class DiscretizationDB(object):

    """Class for dealing with discretization parameters."""

    def __init__(self):

        """
        Constructor of the class DiscretizationDB.
        """

        # Frequency discretization for the hdb.
        self._wave_frequencies = None
        self._max_frequency = None
        self._min_frequency = None
        self._nb_frequencies = None

        # Wave directions discretization for the hdb.
        self._wave_dirs = None
        self._max_angle = None
        self._min_angle = None
        self._nb_wave_directions = None

        # Time discretization for the impulse response functions.
        self._final_time = 100
        self._nb_time_sample = None
        self._delta_time = 0.008

    @property
    def max_frequency(self):

        """This function gives the maximum frequency.

        Returns
        -------
        float
            Maximum frequency.
        """

        return self._max_frequency

    @property
    def min_frequency(self):

        """This function gives the minimum frequency.

        Returns
        -------
        float
            Minimum frequency.
        """

        return self._min_frequency

    @property
    def nb_frequencies(self):

        """This function gives the number of frequencies.

        Returns
        -------
        int
            Number of frequencies.
        """

        return self._nb_frequencies

    @nb_frequencies.setter
    def nb_frequencies(self, value):

        """This function sets the number of frequencies.

        Parameter
        ----------
        int : value
            Number of frequencies.
        """

        if isinstance(value, int):
            self._nb_frequencies = value
            print("Warning : Number of wave frequencies sets to %i." % value)
        else:
            print("Warning : Value must be an integer.")

    @property
    def final_time(self):

        """This function gives the final time.

        Returns
        -------
        float
            Final time.
        """

        return self._final_time

    @final_time.setter
    def final_time(self, value):

        """ This function gives the final time the time discretizations

        Parameter:
        ---------
        float value:
            Final time (in second)
        """

        if value > -1e-8:
            self._final_time = value
        else:
            print("Warning : Final time must be >= 0.")

    @property
    def nb_time_sample(self):

        """This function gives the number of discretizations.

        Returns
        -------
        int
            Number of discretizations.
        """

        return self._nb_time_sample

    @nb_time_sample.setter
    def nb_time_sample(self, value):

        """ This function sets the number of time samples

        Parameter:
        ---------
        int value:
            Number of time sample
        """

        if isinstance(value, int) and value >= 0:
            self._nb_time_sample = value
        else:
            print("Warning : Number of time simples must be an integer >= 0.")

    @property
    def delta_time(self):

        """ This function gives the size of the time discretizations

        Returns
        -------
        float
            Delta time of the discretization (in second)

        """

        return self._delta_time

    @delta_time.setter
    def delta_time(self, value):
        """ This function sets the delta time size discretization

        Parameter:
        ---------
        float
            Delta time of the time discretization (in second)
        """

        if value > -1e-8:
            self._delta_time = value
        else:
            print("Warning : Delta time must be > 0.")

    @property
    def max_angle(self):

        """This function gives the maximum wave direction.

        Returns
        -------
        float
            Maximum wave direction.
        """

        return self._max_angle

    @property
    def min_angle(self):

        """This function gives the minimum wave direction.

        Returns
        -------
        float
            Minimum wave direction.
        """

        return self._min_angle

    @property
    def nb_wave_directions(self):

        """This function gives the number of wave directions.

        Returns
        -------
        int
            Number of wave directions.
        """

        return self._nb_wave_directions

    @nb_wave_directions.setter
    def nb_wave_directions(self, value):

        """This function sets the number of wave directions.

        Parameter
        ----------
        int : value
            Number of wave directions.
        """

        if isinstance(value, int):
            self._nb_wave_directions = value
            print("Warning : Number of wave directions sets to %i." % value)
        else:
            print("Warning : Dimension must be an integer.")

    @property
    def wave_dirs(self):

        """This function gives all the wave directions.

        Returns
        -------
        Array of floats
            Wave directions.
        """

        return self._wave_dirs

    @property
    def wave_frequencies(self):

        """This function gives all the frequencies.

        Returns
        -------
        Array of floats
            Frequencies.
        """

        return self._wave_frequencies

    @property
    def time(self):

        """This function gives the time vector.

        Returns
        -------
        Array of floats
            Time vector.
        """

        return np.linspace(0., self._final_time, self._nb_time_sample)

    def initialize(self, pyHDB):

        """This function peforms the initialization of the dicretization from the hydrodynamic database.

        Returns
        -------
        HydroDB
            Hydrodynamic database.
        """

        print("")
        print("-- Initialize --")

        # Wave frequencies.
        if self.max_frequency is None:
            self._max_frequency = pyHDB.max_wave_freq

        if self.min_frequency is None:
            self._min_frequency = pyHDB.min_wave_freq

        if self.nb_frequencies is None:
            self._nb_frequencies = pyHDB.nb_wave_freq

        self._wave_frequencies = np.linspace(self.min_frequency, self.max_frequency, self.nb_frequencies)

        print(" Max frequency : %16.8f" % self._min_frequency)
        print(" Min frequency : %16.8f" % self._max_frequency)
        print(" Nb Wave frequencies : %i" % self._nb_frequencies)

        # Wave direction.
        if self.max_angle is None:
            self._max_angle = pyHDB.max_wave_dir

        if self.min_angle is None:
            self._min_angle = pyHDB.min_wave_dir

        if self.nb_wave_directions is None:
            self._nb_wave_directions = pyHDB.nb_wave_dir

        print(" Angle max : %16.8f" % self._max_angle)
        print(" Angle min : %16.8f" % self._min_angle)
        print(" Nb Wave direction : %i" % self._nb_wave_directions)
        print("")

        self._wave_dirs = np.linspace(self.min_angle, self.max_angle, self.nb_wave_directions)

        # Time.
        if self._nb_time_sample is None:
            self._nb_time_sample = int(self.final_time / self._delta_time) + 1
            self._delta_time = self.final_time / float(self.nb_time_sample - 1)

        return