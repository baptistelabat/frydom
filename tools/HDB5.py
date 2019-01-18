#!/usr/bin/env python
#  -*- coding: utf-8 -*-
"""Module to create a hydrodynamic database for frydom"""

import os, h5py
from math import *
import numpy as np

from HydroDB.bem_reader import NemohReader
from HydroDB.body_db import BodyDB

from environment_db import EnvironmentDB
from discretization_db import DiscretizationDB

from scipy import interpolate


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


class HDB5(object):

    def __init__(self):

        self._hdb = None
        self._environment = EnvironmentDB()
        self._discretization = DiscretizationDB()
        self._bodies = []
        self._wave_direction = np.array([])
        self._wave_frequencies = np.array([])

        return

    def nemoh_reader(self, input_directory='.', nb_faces_by_wavelength=None):

        if not os.path.isabs(input_directory):
            input_directory = os.path.abspath(input_directory)

        # Verifying there is the Nemoh.cal file inside input_directory
        nemoh_cal_file = os.path.join(input_directory, 'Nemoh.cal')
        if not os.path.isfile(nemoh_cal_file):
            raise AssertionError('Folder %s seems not to be a Nemoh calculation folder as '
                                 'we did not find Nemoh.cal' % input_directory)

        print('========================')
        print('Reading Nemoh results...')
        print('========================')

        if nb_faces_by_wavelength is None:
            nb_faces_by_wavelength = 10

        reader = NemohReader(cal_file=nemoh_cal_file, test=True, nb_face_by_wave_length=nb_faces_by_wavelength)

        # Save the whole database
        self._hdb = reader.hydro_db

        # Save the environment
        self._environment.load_data(reader.hydro_db)

        # Save the discretization
        #self._discretization.load_data(reader.hydro_db)

        # Load bodies
        for i_body in range(reader.hydro_db.body_mapper.nb_bodies):
            self._bodies.append(BodyDB(self._hdb, i_body))

        print('-------> Nemoh data successfully loaded from "%s"' % input_directory)

    def get_body_list(self):
        for body in self._bodies:
            print("body id: %i, name : %s" % (body.id, body.name))
        return

    def get_body(self, id=None, name=None):

        if name:
            return self._find_body_by_name(name)
        elif id:
            return self._bodies[id]
        else:
            print("warning : id or name must be defined")

        return None

    @property
    def body(self):
        return self._bodies

    def _find_body_by_name(self, name):

        for body in self._bodies:
            if body.mesh.name == name:
                return body

        print("warning : no body found with name %s" % name)
        return None

    def _initialize(self):

        # Compute froude krylov
        self._hdb.froude_krylov_db

        # Compute diffraction
        self._hdb.diffraction_db

        self._initialize_wave_dir()

        self._discretization.initialize(self._hdb)

        self._hdb.radiation_db.eval_impulse_response_function(tf=100, dt=0.1)

        self._hdb.radiation_db.eval_infinite_added_mass()

        self._hdb.radiation_db.eval_impulse_response_function_Ku(tf=100, dt=0.1)

        for body in self._bodies:

            if body.wave_drift:
                body.wave_drift.discrete_wave_dir = self._discretization.wave_dirs * pi/ 180.
                body.wave_drift.initialize()

        return

    def _initialize_wave_dir(self):

        self._hdb._wave_dirs = np.linspace(self._hdb.min_wave_dir, self._hdb.max_wave_dir, self._hdb.nb_wave_dir)

        fk_db = self._hdb.froude_krylov_db
        diff_db = self._hdb.diffraction_db

        # --- Adjust convention of wage direction to GOTO
        if self._hdb.min_wave_dir >= -np.float32() and self._hdb.max_wave_dir <= 180. + np.float32():
            self._hdb._wave_dirs = 180. - self._hdb._wave_dirs
            self._hdb._wave_dirs, fk_db, diff_db = symetrize(self._hdb._wave_dirs, fk_db, diff_db)
        else:
            self._hdb._wave_dirs = np.fmod(self._hdb._wave_dirs + 180., 360.)

        n180 = 0
        i360 = -9
        for idir in range(self._hdb._wave_dirs.size):
            wave_dir = self._hdb._wave_dirs[idir]

            if abs(wave_dir) < 0.01:
                i360 = idir
            elif abs(wave_dir - 180) < 0.01:
                n180 += 1
                if n180 == 2:
                    self._hdb._wave_dirs[idir] = 360.
                    fk_db.data[:, :, idir] = fk_db.data[:, :, i360]
                    diff_db.data[:, :, idir] = diff_db.data[:, :, i360]

        # -- sort direction
        sort_dirs = np.argsort(self._hdb._wave_dirs)
        self._hdb._wave_dirs = self._hdb._wave_dirs[sort_dirs]
        self._hdb._froude_krylov_db.data = fk_db.data[:, :, sort_dirs]
        self._hdb._diffraction_db.data = diff_db.data[:, :, sort_dirs]
        #self._hdb._wave_dirs = self._wave_dirs

        self._max_angle = np.max(self._hdb._wave_dirs)
        self._min_angle = np.min(self._hdb._wave_dirs)
        self._nb_wave_directions = self._hdb._wave_dirs.shape[0]

        return

    def set_direction(self, d_angle, unit='deg'):

        if unit == 'deg':
            d_angle *= pi/180.

        n = int(2.*pi/d_angle)
        d_angle = 2.*pi / float(n)

        self._wave_direction = np.linspace(0, 2.*pi, d_angle)

        return

    def set_frequencies(self, f_min, f_max, df, unit='rads'):
        self._discretization.set_wave_frequencies(f_min, f_max, df, unit)

    def set_directions(self, min_angle, max_angle, delta_angle, unit='rad'):
        self._discretization.set_wave_direction(min_angle, max_angle, delta_angle, unit)

    def set_directions(self, delta_angle, unit='rad'):

        if unit == 'deg':
            delta_angle *= pi/180.

        self._discretization.set_wave_direction(0., 2.*pi, delta_angle, 'rad')

        return

    def _interpolate_diffraction(self):

        f_interp_dir = interpolate.interp1d(self._hdb.wave_dirs, self._hdb.diffraction, axis=2)
        self._hdb.diffraction = f_interp_dir(self._discretization.wave_dirs)

        f_interp_freq = interpolate.interp1d(self._hdb.wave_frequencies, self._hdb.diffraction, axis=1)
        self._hdb.diffraction = f_interp_freq(self._discretization.wave_frequencies)

        return

    def _interpolate_froude_krylov(self):

        f_interp_dir = interpolate.interp1d(self._hdb.wave_dirs, self._hdb.froude_krylov, axis=2)
        self._hdb.froude_krylov = f_interp_dir(self._wave_direction)

        f_interp_freq = interpolate.interp1d(self._hdb.wave_frequencies, self._hdb.froude_krylov, axis=1)
        self._hdb.froude_krylov = f_interp_dir(self._discretization.wave_frequencies)

        return

    def write_hdb5(self, output_file=None):

        print('========================')
        print('Intialize HDB5 database...')
        print('========================')

        self._initialize()

        print('========================')
        print('Writing HDB5 database...')
        print('========================')

        if output_file is None:
            hdb5_file = os.path.abspath('frydom.hdb5')

        else:
            # Verifying that the output file has the extension .hdb5
            root, ext = os.path.splitext(output_file)
            if not ext == '.hdb5':
                raise IOError('Please register the output file with a .hdb5 extension')

            hdb5_file = output_file

            if not os.path.isabs(output_file):
                hdb5_file = os.path.abspath(hdb5_file)

        try:

            with h5py.File(hdb5_file, 'w') as writer:
                self._environment.write_hdb5(writer)
                self._discretization.write_hdb5(writer)
                for body in self._bodies:
                    body.write_hdb5(writer)

        except IOError:
            raise IOError('Problem in writing HDB5 file at location %s' % hdb5_file)

        print('-------> "%s" has been written' % hdb5_file)








