#!/usr/bin/env python
#  -*- coding: utf-8 -*-
"""Module to create the discretization database for frydom hydrodynamic database"""

from math import *
import numpy as np


def symetrize(wave_dirs, fk_db, diff_db):

    [nmode, nbody, ndir] = fk_db.data.shape

    for i in range(ndir):

        if wave_dirs[i] > np.float32(0.):

            # New wave direction
            new_dir = -wave_dirs[i] % 360
            if new_dir < 0:
                new_dir += 360.

            # Add corresponding data
            wave_dirs = np.append(wave_dirs, new_dir)

            fk_db_temp = np.copy(fk_db.data[:, :, i])
            fk_db_temp[(1, 4, 5), :] = -fk_db_temp[(1, 4, 5), :]

            fk_db.data = np.concatenate((fk_db.data, fk_db_temp.reshape(nmode, nbody, 1)), axis=2)

            diff_db_temp = np.copy(diff_db.data[:, :, i])
            diff_db_temp[(1, 4, 5), :] = -diff_db_temp[(1, 4, 5), :]
            diff_db.data = np.concatenate((diff_db.data, diff_db_temp.reshape(nmode, nbody, 1)), axis=2)

    return wave_dirs, fk_db, diff_db


class DiscretizationDB(object):

    def __init__(self):
        # Frequency discretization
        self._wave_frequencies = None
        self._max_frequency = None
        self._min_frequency = None
        self._nb_frequencies = None
        # Time discretization
        self._final_time = None
        self._nb_time_sample = 0
        # Wave directions discretization
        self._wave_dirs = None
        self._max_angle = None
        self._min_angle = None
        self._nb_wave_directions = None

    @property
    def max_frequency(self):
        return self._max_frequency

    @property
    def min_frequency(self):
        return self._min_frequency

    @property
    def nb_frequencies(self):
        return self._nb_frequencies

    @property
    def final_time(self):
        return self._final_time

    @property
    def nb_time_sample(self):
        return self._nb_time_sample

    @property
    def max_angle(self):
        return self._max_angle

    @property
    def min_angle(self):
        return self._min_angle

    @property
    def nb_wave_directions(self):
        return self._nb_wave_directions

    @nb_wave_directions.setter
    def nb_wave_directions(self, value):
        if isinstance(value, int):
            self._nb_wave_directions = value
            print("warning : wave dir set to %i" % value)
        else:
            print("warning : dimension must be an integer")

    @property
    def wave_dirs(self):
        return self._wave_dirs

    @property
    def wave_frequencies(self):
        return self._wave_frequencies

    def initialize(self, hdb):

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

        time = hdb.radiation_db.get_irf_db().time
        self._final_time = time[-1]
        self._nb_time_sample = len(time)

        return

    def set_wave_frequencies(self, f_min, f_max, df, unit='rads'):

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

        if unit == 'deg':
            min_angle *= pi/180.
            max_angle *= pi/180.
            delta_angle *= pi/180.

        n = int((max_angle - min_angle)/delta_angle)
        delta_angle = (max_angle - min_angle) / float(n)

        self._wave_dirs = np.arange(min_angle, max_angle, delta_angle)

        return

    def write_hdb5(self, writer):

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

        return

    def compute_wave_dirs(self, hdb):

        self._wave_dirs = np.linspace(hdb.min_wave_dir, hdb.max_wave_dir, hdb.nb_wave_dir)

        fk_db = hdb.froude_krylov_db
        diff_db = hdb.diffraction_db

        # --- Adjust convention of wage direction to GOTO
        if hdb.min_wave_dir >= -np.float32() and hdb.max_wave_dir <= 180. + np.float32():
            self._wave_dirs = 180. - self._wave_dirs
            self._wave_dirs,  fk_db, diff_db = symetrize(self._wave_dirs, fk_db, diff_db)
        else:
            self._wave_dirs = np.fmod(self._wave_dirs + 180., 360.)

        n180 = 0
        i360 = -9
        for idir in range(self._wave_dirs.size):
            wave_dir = self._wave_dirs[idir]

            if abs(wave_dir) < 0.01:
                i360 = idir
            elif abs(wave_dir - 180) < 0.01:
                n180 += 1
                if n180 == 2:
                    self._wave_dirs[idir] = 360.
                    fk_db.data[:, :, idir] = fk_db.data[:, :, i360]
                    diff_db.data[:, :, idir] = diff_db.data[:, :, i360]

        # -- sort direction
        sort_dirs = np.argsort(self._wave_dirs)
        self._wave_dirs = self._wave_dirs[sort_dirs]
        hdb._froude_krylov_db.data = fk_db.data[:, :, sort_dirs]
        hdb._diffraction_db.data = diff_db.data[:, :, sort_dirs]
        hdb._wave_dirs = self._wave_dirs

        self._max_angle = np.max(self._wave_dirs)
        self._min_angle = np.min(self._wave_dirs)
        self._nb_wave_directions = self._wave_dirs.shape[0]

        return
