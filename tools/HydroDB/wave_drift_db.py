#!/usr/bin/env python
#  -*- coding: utf-8 -*-
"""Module to load wave drift polar coefficient"""

import numpy as np
from math import *
from scipy.interpolate import interp1d


class CoeffData(object):

    def __init__(self):
        self.name = None
        self.heading = []
        self.freq = []
        self.data = []
        self.heading_index = None
        return


class WaveDriftDB(object):

    def __init__(self):
        self._modes = dict()
        self._heading_index = None
        self._sym_x = False
        self._sym_y = False
        self._nb_frequencies = None
        self._discrete_frequency = None
        self._discrete_wave_dir = None

    @property
    def modes(self):
        return self._modes

    @property
    def sym_x(self):
        return self._sym_x

    @sym_x.setter
    def sym_x(self, value):
        if isinstance(value, bool):
            self._sym_x = value
        else:
            print("warning : wrong type value, must be a boolean")

    @property
    def sym_y(self):
        return self._sym_y

    @sym_y.setter
    def sym_y(self, value):
        if isinstance(value, bool):
            self._sym_y = value
        else:
            print("warning : wrong type value, must be a boolean")

    @property
    def heading_index(self):
        return self._heading_index

    @property
    def nb_frequencies(self):
        return self._nb_frequencies

    @nb_frequencies.setter
    def nb_frequencies(self, value):
        self._nb_frequencies = value

    @property
    def discrete_wave_dir(self):
        return self._discrete_wave_dir

    @discrete_wave_dir.setter
    def discrete_wave_dir(self, value):
        if isinstance(value, np.ndarray):
            if value.ndim == 1:
                self._discrete_wave_dir = value
            else:
                print("warning : dimension must be equal to 1")
        else:
            print("warning : type must be nd array")

    @property
    def discrete_frequency(self):
        return self._discrete_frequency

    def scaling_mode(self, mode, factor):
        self._modes[mode].data[:] *= factor

    def add(self, mode, freq, data, wave_dir, unit_freq='rads', unit_dir='rad'):

        if unit_freq == 'Hz':
            freq = (2.*pi)*freq
        elif unit_freq == 's':
            freq = (2.*pi) / freq[::-1]
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
        self.add('surge', freq, data, wave_dir, unit_freq, unit_dir)

    def add_cy(self, freq, data, wave_dir, unit_freq='rads', unit_dir='rad'):
        self.add('sway', freq, data, wave_dir, unit_freq, unit_dir)

    def add_cz(self, freq, data, wave_dir, unit_freq='rads', unit_dir='rad'):
        self.add('heave', freq, data, wave_dir, unit_freq, unit_dir)

    def add_cr(self, freq, data, wave_dir, unit_freq='rads', unit_dir='rad'):
        self.add('roll', freq, data, wave_dir, unit_freq, unit_dir)

    def add_cm(self, freq, data, wave_dir, unit_freq='rads', unit_dir='rad'):
        self.add('pitch', freq, data, wave_dir, unit_freq, unit_dir)

    def add_cn(self, freq, data, wave_dir, unit_freq='rads', unit_dir='rad'):
        self.add('yaw', freq, data, wave_dir, unit_freq, unit_dir)

    def get_min_frequency(self):

        f_min = -999.
        for key, mode in self._modes.items():
            f_min = max(f_min, max([val[0] for val in mode.freq]))

        return f_min

    def get_max_frequency(self):

        f_max = 999.
        for key, mode in self._modes.items():
            f_max = min(f_max, min([val[-1] for val in mode.freq]))

        return f_max

    def _set_discrete_frequency(self):
        f_min = self.get_min_frequency()
        f_max = self.get_max_frequency()
        self._discrete_frequency = np.linspace(f_min, f_max, self._nb_frequencies)
        return

    def initialize(self):

        self._set_discrete_frequency()

        for key in self._modes:

            headings, self._modes[key].heading_index = np.unique(self._modes[key].heading, return_index=True)

            for i in range(len(self._modes[key].freq)):
                freq = self._modes[key].freq[i]
                data = self._modes[key].data[i]
                self._modes[key].data[i] = np.interp(self._discrete_frequency, freq, data)
                self._modes[key].freq[i] = self._discrete_frequency

                if abs(self._modes[key].freq[i][0]) < 1e-2:
                    self._modes[key].freq[i][0] = 0.

                if abs(self._modes[key].freq[i][-1] - pi) < 1e-2:
                    self._modes[key].freq[i][-1] = pi

            x = np.array(self._modes[key].heading)
            y = np.array(self._modes[key].data)

            f_interp = interp1d(x, y, kind='linear', axis=0, fill_value='extrapolate', bounds_error=False)
            data = f_interp(self._discrete_wave_dir)

            self._modes[key].data = data
            self._modes[key].heading = list(self._discrete_wave_dir)

        return

    def get_structure(self):

        for key, mode in self._modes.items():

            for i_dir, angle in enumerate(mode.heading):

                print("mode: %s ;heading_%i:%16.8f ;size_freq(%s)" % (key, i_dir, angle * 180./pi, str(mode.data[i_dir].size)))

