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
"""Module to load wave drift polar coefficients"""

import numpy as np
from scipy.interpolate import interp1d


class CoeffData(object):
    """
        Class for dealing with wave drift polar coefficients.
    """

    def __init__(self):
        """
        Constructor of the class CoeffData.
        """

        self.name = None
        self.heading = []
        self.freq = []
        self.data = []
        self.heading_index = None
        return


class WaveDriftDB(object):
    """
    Class for dealing with wave drift polar coefficients.
    """

    def __init__(self):

        """
        Constructor of the class WaveDriftDB.
        """

        self._modes = dict()
        self._heading_index = None
        self._sym_x = False
        self._sym_y = False
        self._nb_frequencies = None
        self._discrete_frequency = None
        self._discrete_wave_dir = None

    @property
    def modes(self):

        """This function gives the modes of the polar coefficients.

        Returns
        -------
        dict
            Dictionary of the modes.
        """

        return self._modes

    @property
    def sym_x(self):

        """This function indicates if a symmetry in the plane xOZ is present.

        Returns
        -------
        bool
            True if a symmetry along x is present, false otherwise.
        """

        return self._sym_x

    @sym_x.setter
    def sym_x(self, value):

        """This function sets presence of a symmetry along x.

        Parameter
        ----------
        float : value
            True if a symmetry along x is present, false otherwise.
        """

        if isinstance(value, bool):
            self._sym_x = value
        else:
            print("warning : wrong type value, must be a boolean")

    @property
    def sym_y(self):

        """This function indicates if a symmetry along y is present.

        Returns
        -------
        bool
            True if a symmetry along y is present, false otherwise.
        """

        return self._sym_y

    @sym_y.setter
    def sym_y(self, value):

        """This function sets presence of a symmetry along y.

        Parameter
        ----------
        float : value
            True if a symmetry along y is present, false otherwise.
        """

        if isinstance(value, bool):
            self._sym_y = value
        else:
            print("warning : wrong type value, must be a boolean")

    @property
    def heading_index(self):

        """This function gives the heading index.

        Returns
        -------
        int
            Heading index.
        """

        return self._heading_index

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

        self._nb_frequencies = value

    @property
    def discrete_wave_dir(self):

        """This function gives the wave direction discretization.

        Returns
        -------
        Array of floats
            Wave direction discretization.
        """

        return self._discrete_wave_dir

    @discrete_wave_dir.setter
    def discrete_wave_dir(self, value):

        """This function sets the wave direction vector.

        Parameter
        ----------
        Array of floats : value
            Wave direction vector for the discretization.
        """

        if isinstance(value, np.ndarray):
            if value.ndim == 1:
                self._discrete_wave_dir = value
            else:
                print("warning : dimension must be equal to 1")
        else:
            print("warning : type must be nd array")

    @property
    def discrete_frequency(self):

        """This function gives the frequency discretization.

        Returns
        -------
        Array of floats
            Frequency discretization.
        """

        return self._discrete_frequency

    @discrete_frequency.setter
    def discrete_frequency(self, value):

        """This function sets the wave frequency vector.

        Parameter
        ----------
        Array of floats : value
            Wave frequency vector for the discretization.
        """

        if isinstance(value, np.ndarray):
            if value.ndim == 1:
                self._discrete_frequency = value
            else:
                print("warning : dimension must be equal to 1")
        else:
            print("warning : type must be nd array")

    def scaling_mode(self, mode, factor):

        """This function scales the polar coefficients.

        Parameters
        ----------
        string : mode
            Mode.
        float : factor
            Scaling factor.
        """

        self._modes[mode].data[:] *= factor

    def add(self, mode, freq, data, wave_dir, unit_freq='rads', unit_dir='rad'):

        """This function adds the polar coefficients.

        Parameters
        ----------
        string : mode
            Mode.
        Array of floats : freq
            Frequency vector.
        Array of floats : data
            Wave drift polar coefficients.
        Array of floats : wave_dir
            Wave directions.
        string : unit_freq, optional
            Frequency unit, by default rad/s.
        string : unit_dir
            wave direction unit, by default rad.
        """

        if unit_freq == 'Hz':
            freq = (2. * pi) * freq
        elif unit_freq == 's':
            freq = (2. * pi) / freq[::-1]
            data = data[::-1]

        if unit_dir == "deg":
            wave_dir = np.radians(wave_dir)

        if mode not in self._modes.keys():
            self._modes[mode] = CoeffData()
            self._modes[mode].name = mode

        self._modes[mode].heading.append(wave_dir)
        self._modes[mode].freq.append(freq)
        self._modes[mode].data.append(data)

        return

    def add_cx(self, freq, data, wave_dir, unit_freq='rads', unit_dir='rad'):

        """This function adds the cx (surge) polar coefficients.

        Parameters
        ----------
        string : mode
            Mode.
        Array of floats : freq
            Frequency vector.
        Array of floats : data
            Wave drift polar coefficients.
        Array of floats : wave_dir
            Wave directions.
        string : unit_freq, optional
            Frequency unit, by default rad/s.
        string : unit_dir
            wave direction unit, by default rad.
        """

        self.add('surge', freq, data, wave_dir, unit_freq, unit_dir)

    def add_cy(self, freq, data, wave_dir, unit_freq='rads', unit_dir='rad'):

        """This function adds the cy (sway) polar coefficients.

        Parameters
        ----------
        string : mode
            Mode.
        Array of floats : freq
            Frequency vector.
        Array of floats : data
            Wave drift polar coefficients.
        Array of floats : wave_dir
            Wave directions.
        string : unit_freq, optional
            Frequency unit, by default rad/s.
        string : unit_dir
            wave direction unit, by default rad.
        """

        self.add('sway', freq, data, wave_dir, unit_freq, unit_dir)

    def add_cz(self, freq, data, wave_dir, unit_freq='rads', unit_dir='rad'):

        """This function adds the cz (heave) polar coefficients.

        Parameters
        ----------
        string : mode
            Mode.
        Array of floats : freq
            Frequency vector.
        Array of floats : data
            Wave drift polar coefficients.
        Array of floats : wave_dir
            Wave directions.
        string : unit_freq, optional
            Frequency unit, by default rad/s.
        string : unit_dir
            wave direction unit, by default rad.
        """

        self.add('heave', freq, data, wave_dir, unit_freq, unit_dir)

    def add_cr(self, freq, data, wave_dir, unit_freq='rads', unit_dir='rad'):

        """This function adds the cr (roll) polar coefficients.

        Parameters
        ----------
        string : mode
            Mode.
        Array of floats : freq
            Frequency vector.
        Array of floats : data
            Wave drift polar coefficients.
        Array of floats : wave_dir
            Wave directions.
        string : unit_freq, optional
            Frequency unit, by default rad/s.
        string : unit_dir
            wave direction unit, by default rad.
        """

        self.add('roll', freq, data, wave_dir, unit_freq, unit_dir)

    def add_cm(self, freq, data, wave_dir, unit_freq='rads', unit_dir='rad'):

        """This function adds the cm (pitch) polar coefficients.

        Parameters
        ----------
        string : mode
            Mode.
        Array of floats : freq
            Frequency vector.
        Array of floats : data
            Wave drift polar coefficients.
        Array of floats : wave_dir
            Wave directions.
        string : unit_freq, optional
            Frequency unit, by default rad/s.
        string : unit_dir
            wave direction unit, by default rad.
        """

        self.add('pitch', freq, data, wave_dir, unit_freq, unit_dir)

    def add_cn(self, freq, data, wave_dir, unit_freq='rads', unit_dir='rad'):

        """This function adds the cn (yaw) polar coefficients.

        Parameters
        ----------
        string : mode
            Mode.
        Array of floats : freq
            Frequency vector.
        Array of floats : data
            Wave drift polar coefficients.
        Array of floats : wave_dir
            Wave directions.
        string : unit_freq, optional
            Frequency unit, by default rad/s.
        string : unit_dir
            wave direction unit, by default rad.
        """

        self.add('yaw', freq, data, wave_dir, unit_freq, unit_dir)

    def get_min_frequency(self):

        """This function gives the minimum frequency.

        Returns
        -------
        Floats
            Minimum frequency.
        """

        f_min = -999.
        for key, mode in self._modes.items():
            f_min = max(f_min, max([val[0] for val in mode.freq]))

        return f_min

    def get_max_frequency(self):

        """This function gives the maximum frequency.

        Returns
        -------
        Floats
            Maximum frequency.
        """

        f_max = 999.
        for key, mode in self._modes.items():
            f_max = min(f_max, min([val[-1] for val in mode.freq]))

        return f_max

    def _set_discrete_frequency(self):

        """This function sets the frequency vector."""

        f_min = self.get_min_frequency()
        f_max = self.get_max_frequency()
        self._discrete_frequency = np.linspace(f_min, f_max, self._nb_frequencies)
        return

    def initialize(self):

        """
        This function sorts the data with respect to the wave direction according to an increasing order and interpolates the data wrt the frequencies and the wave directions.
        """

        self._set_discrete_frequency()

        for key in self._modes:

            # Sorting the data with respect to the wave directions.
            headings, self._modes[key].heading_index = np.unique(self._modes[key].heading, return_index=True)

            for i in range(len(self._modes[key].freq)):
                freq = self._modes[key].freq[i]
                data = self._modes[key].data[i]

                # Interpolation with respect to the wave frequencies.
                self._modes[key].data[i] = np.interp(self._discrete_frequency, freq, data)
                self._modes[key].freq[i] = self._discrete_frequency

                if abs(self._modes[key].freq[i][0]) < 1e-2:
                    self._modes[key].freq[i][0] = 0.

                if abs(self._modes[key].freq[i][-1] - pi) < 1e-2:
                    self._modes[key].freq[i][-1] = pi

            x = np.array(self._modes[key].heading)
            y = np.array(self._modes[key].data)

            # Interpolation with respect to the wave directions.
            f_interp = interp1d(x, y, kind='linear', axis=0, fill_value='extrapolate', bounds_error=False)
            data = f_interp(self._discrete_wave_dir)

            self._modes[key].data = data
            self._modes[key].heading = list(self._discrete_wave_dir)

        return

    def get_structure(self):

        """This function prints the modes, the headings and the frequencies."""

        for key, mode in self._modes.items():

            for i_dir, angle in enumerate(mode.heading):
                print("mode: %s ;heading_%i:%16.8f ;size_freq(%s)" % (
                key, i_dir, np.degrees(angle), str(mode.data[i_dir].size)))
