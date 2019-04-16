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

        # Frequency discretization
        self._wave_frequencies = None
        self._max_frequency = None
        self._min_frequency = None
        self._nb_frequencies = None
        # Time discretization
        self._final_time = None
        self._nb_time_sample = None
        self._delta_time = None
        # Wave directions discretization
        self._wave_dirs = None
        self._max_angle = None
        self._min_angle = None
        self._nb_wave_directions = None

    @property
    def max_frequency(self):

        """This subroutine gives the maximum frequency.

        Returns
        -------
        float
            Maximum frequency.
        """

        return self._max_frequency

    @property
    def min_frequency(self):

        """This subroutine gives the minimum frequency.

        Returns
        -------
        float
            Minimum frequency.
        """

        return self._min_frequency

    @property
    def nb_frequencies(self):

        """This subroutine gives the number of frequencies.

        Returns
        -------
        int
            Number of frequencies.
        """

        return self._nb_frequencies

    @nb_frequencies.setter
    def nb_frequencies(self, value):

        """This subroutine sets the number of frequencies.

        Parameter
        ----------
        int : value
            Number of frequencies.
        """

        if isinstance(value, int):
            self._nb_frequencies = value
        else:
            print("warning : value must be an integer")

    @property
    def final_time(self):

        """This subroutine gives the final time.

        Returns
        -------
        float
            Final time.
        """

        return self._final_time

    @final_time.setter
    def final_time(self, value):
        """ This subroutines gives the final time the time discretizations

        Parameter:
        ---------
        float value:
            Final time (in second)
        """

        if value > -1e-8:
            self._final_time = value
        else:
            print("warning : final time must be >= 0")

    @property
    def nb_time_sample(self):

        """This subroutine gives the number of discretizations.

        Returns
        -------
        int
            Number of discretizations.
        """

        return self._nb_time_sample

    @nb_time_sample.setter
    def nb_time_sample(self, value):
        """ This subroutine set the number of time samples

        Parameter:
        ---------
        int value:
            Number of time sample
        """

        if isinstance(value, int) and value >= 0:
            self._nb_time_sample = value
        else:
            print("warning : nb of time simple must be an integer >= 0")

    @property
    def delta_time(self):

        """ This subroutine gives the size of the time discretizations

        Returns
        -------
        float
            Delta time of the discretization (in second)

        """

        return self._delta_time

    @delta_time.setter
    def delta_time(self, value):
        """ This subroutines set the delta time size discretization

        Parameter:
        ---------
        float
            Delta time of the time discretization (in second)
        """

        if value > -1e-8:
            self._delta_time = value
        else:
            print("warning : delta time muse be >0")

    @property
    def max_angle(self):

        """This subroutine gives the maximum wave direction.

        Returns
        -------
        float
            Maximum wave direction.
        """

        return self._max_angle

    @property
    def min_angle(self):

        """This subroutine gives the minimum wave direction.

        Returns
        -------
        float
            Minimum wave direction.
        """

        return self._min_angle

    @property
    def nb_wave_directions(self):

        """This subroutine gives the number of wave directions.

        Returns
        -------
        int
            Number of wave directions.
        """

        return self._nb_wave_directions

    @nb_wave_directions.setter
    def nb_wave_directions(self, value):

        """This subroutine sets the number of wave directions.

        Parameter
        ----------
        int : value
            Number of wave directions.
        """

        if isinstance(value, int):
            self._nb_wave_directions = value
            print("warning : wave dir set to %i" % value)
        else:
            print("warning : dimension must be an integer")

    @property
    def wave_dirs(self):

        """This subroutine gives all the wave directions.

        Returns
        -------
        Array of floats
            Wave directions.
        """

        return self._wave_dirs

    @property
    def wave_frequencies(self):

        """This subroutine gives all the frequencies.

        Returns
        -------
        Array of floats
            Frequencies.
        """

        return self._wave_frequencies

    @property
    def time(self):

        """This subroutine gives the time vector.

        Returns
        -------
        Array of floats
            Time vector.
        """

        return np.linspace(0., self._final_time, self._nb_time_sample)

    def initialize(self, hdb):

        """This subroutine peform the initialization of the dicretization from the hydrodynamic database.

        Returns
        -------
        HydroDB
            Hydrodynamic database.
        """

        print("")
        print("-- Initialize --")

        # Wave frequency

        if self.max_frequency is None:
            self._max_frequency = hdb.max_frequency

        if self.min_frequency is None:
            self._min_frequency = hdb.min_frequency

        if self.nb_frequencies is None:
            self._nb_frequencies = hdb.nb_frequencies

        self._wave_frequencies = np.linspace(self.min_frequency, self.max_frequency, self.nb_frequencies)

        print(" Max frequency : %16.8f" % self._min_frequency)
        print(" Min frequency : %16.8f" % self._max_frequency)
        print(" Nb Wave frequencies : %i" % self._nb_frequencies)

        # Wave direction

        if self.max_angle is None:
            self._max_angle = hdb.max_wave_dir

        if self.min_angle is None:
            self._min_angle = hdb.min_wave_dir

        if self.nb_wave_directions is None:
            self._nb_wave_directions = hdb.nb_wave_dir

        print(" Angle max : %16.8f" % self._max_angle)
        print(" Angle min : %16.8f" % self._min_angle)
        print(" Nb Wave direction : %i" % self._nb_wave_directions)

        self._wave_dirs = np.linspace(self.min_angle, self.max_angle, self.nb_wave_directions)

        # Time

        if self._final_time is None and self._nb_time_sample is None:
            time = hdb.radiation_db.get_irf_db().time
            self._final_time = time[-1]
            self._nb_time_sample = len(time)
            self._delta_time = time[1] - time[0]

        elif self._delta_time is None:
            self._delta_time = self.final_time / (self.nb_time_sample - 1)

        elif self._nb_time_sample is None:
            self._nb_time_sample = int(self.final_time / self._delta_time) + 1
            self._delta_time = self.final_time / (self.nb_time_sample - 1)

        return

    def set_wave_frequencies(self, f_min, f_max, df, unit='rads'):

        """This subroutine sets the wave frequencies (still used?).

        Parameters
        ----------
        f_min : float
            Minimum frequency.
        f_max : float
            Maximum frequency.
        df : float
            Frequency step.
        unit : string, optional
            Unit of the angular step: 'rads' (by default) or 'Hz'.
        """

        if unit == 'Hz':
            f_min *= 2.*pi
            f_max *= 2.*pi
            df *= 2.*pi

        self._min_frequency = f_min
        self._max_frequency = f_max

        n = int((f_max - f_min) / df)
        df = (f_max - f_min) / float(n)

        self._wave_frequencies = np.arange(f_min, f_max, df)

        return

    def set_wave_direction(self, min_angle, max_angle, delta_angle, unit='rad'):

        """This subroutine sets the wave directions (still used?).

        Parameters
        ----------
        min_angle : float
            Minimum wave direction.
        max_angle : float
            Maximum wave direction.
        delta_angle : float
            Wave direction angular step.
        unit : string, optional
            Unit of the angular step: 'rad' (by default) or 'deg'.
        """

        if unit == 'deg':
            min_angle *= pi/180.
            max_angle *= pi/180.
            delta_angle *= pi/180.

        n = int((max_angle - min_angle)/delta_angle)
        delta_angle = (max_angle - min_angle) / float(n)

        self._wave_dirs = np.arange(min_angle, max_angle, delta_angle)

        return

    def write_hdb5(self, writer):

        """This subroutine writes the discretization data into the *.hdb5 file.

        Parameters
        ----------
        Writer : string
            *.hdb5 file.
        """

        # Frequency discretization

        discretization_path = "/Discretizations"

        writer.create_group(discretization_path)
        frequential_path = discretization_path + "/Frequency"

        dset = writer.create_dataset(frequential_path + "/NbFrequencies", data=self.nb_frequencies)
        dset.attrs['Description'] = "Number of frequencies in the discretization"

        dset = writer.create_dataset(frequential_path + "/MinFrequency", data=self._min_frequency)
        dset.attrs['Unit'] = "rad/s"
        dset.attrs['Description'] = "Minimum frequency specified for the computations"

        dset = writer.create_dataset(frequential_path + "/MaxFrequency", data=self._max_frequency)
        dset.attrs['Unit'] = "rad/s"
        dset.attrs['Description'] = "Maximum frequency specified for the computations"

        # Wave direction discretization

        wave_direction_path = discretization_path + "/WaveDirections"

        dset = writer.create_dataset(wave_direction_path + "/NbWaveDirections", data=self.nb_wave_directions)
        dset.attrs['Description'] = "Number of wave directions in the discretization"

        dset = writer.create_dataset(wave_direction_path + "/MinAngle", data=self.min_angle)
        dset.attrs['Unit'] = "deg"
        dset.attrs['Description'] = "Minimum angle specified for the computations"

        dset = writer.create_dataset(wave_direction_path + "/MaxAngle", data=self._max_angle)
        dset.attrs['Unit'] = "deg"
        dset.attrs['Description'] = "Maximum angle specified for the computations"

        # Time sample

        time_path = discretization_path + "/Time"

        dset = writer.create_dataset(time_path + "/NbTimeSample", data=self._nb_time_sample)
        dset.attrs['Description'] = "Number of time samples"

        dset = writer.create_dataset(time_path + "/FinalTime", data=self._final_time)
        dset.attrs['Unit'] = "s"
        dset.attrs['Description'] = "Final time for the impulse response function"

        return