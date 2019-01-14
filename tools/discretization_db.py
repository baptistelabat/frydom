#!/usr/bin/env python
#  -*- coding: utf-8 -*-
"""Module to create the discretization database for frydom hydrodynamic database"""

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
        self._max_frequency = None
        self._min_frequency = None
        self._nb_frequencies = 0
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

    def load_data(self, hdb):
        self._max_frequency = hdb.max_frequency
        self._min_frequency = hdb.min_frequency
        self._nb_frequencies = hdb.nb_frequencies

        time = hdb.radiation_db.get_irf_db().time
        self._final_time = time[-1]
        self._nb_time_sample = len(time)

        self.compute_wave_dirs(hdb)

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
        dset = writer.create_dataset(wave_direction_path + "/NbWaveDirections", data=self.nb_wave_dir)
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

        self._max_angle = np.max(self._wave_dirs)
        self._min_angle = np.min(self._wave_dirs)
        self._nb_wave_directions = self._wave_dirs.shape[0]

        return
