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
"""
    Module to create a hydrodynamic database for FRyDoM.
"""

import os, h5py
from math import *
import numpy as np

from HydroDB.bem_reader import NemohReader
from HydroDB.body_db import BodyDB

from environment_db import EnvironmentDB
from discretization_db import DiscretizationDB

from scipy import interpolate

def symetrize(wave_dirs, fk_db, diff_db):

    """This subroutine updates the hdb due to a modification of the wave direction convention.

    Parameters
    ----------
    wave_dirs : float
        Wave directions.
    fk_db : float
        Froude-Krylov loads.
    diff_db : float
        Diffraction loads.
    """

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

    """
        Class HDB5 for dealing with *.h5 files.
    """

    def __init__(self):

        """
            Constructor of the class HDB5.
        """

        self._hdb = None
        self._environment = EnvironmentDB() # Initialization of Environment.
        self._discretization = DiscretizationDB() # Parameters for the new discretization of the HDB.
        self._bodies = []
        self._wave_direction = np.array([])
        self._wave_frequencies = np.array([])
        self._is_initialized = False

        return

    @property
    def body(self):

        """This subroutine returns all the bodies.

        Returns
        -------
        BodyDB
        """

        return self._bodies

    @property
    def discretization(self):

        """This subroutine returns the parameters of the discretization.

        Returns
        -------
        DiscretizationDB
        """

        return self._discretization

    def nemoh_reader(self, input_directory='.', nb_faces_by_wavelength=None):

        """This subroutine reads the *.cal file and stores the data.

        Parameters
        ----------
        input_directory : string, optional
            Path to directory of *.cal file.
        nb_faces_by_wavelength : float, optional
            Number of panels per wave length.
        """

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

        # Reading *.cal.
        reader = NemohReader(cal_file=nemoh_cal_file, test=True, nb_face_by_wave_length=nb_faces_by_wavelength)

        # Storing the whole database.
        self._hdb = reader.hydro_db

        # Saving the environment.
        self._environment.load_data(reader.hydro_db)

        # Save the discretization
        #self._discretization.load_data(reader.hydro_db)

        # Load bodies
        for i_body in range(reader.hydro_db.body_mapper.nb_bodies):
            self._bodies.append(BodyDB(self._hdb, i_body))

        print('-------> Nemoh data successfully loaded from "%s"' % input_directory)

    def get_body_list(self):

        """This subroutine gives the name and the number of each body.
        """

        for body in self._bodies:
            print("body id: %i, name : %s" % (body.id, body.name))
        return

    def get_body(self, id=None, name=None):

        """This subroutine returns a body.

        Parameters
        ----------
        id : int, optional
            Number of a body.
        name : float, optional
            Name of a body.

        Returns
        -------
        BodyDB
        """

        if name:
            return self._find_body_by_name(name)
        elif id:
            return self._bodies[id]
        else:
            print("warning : id or name must be defined")

        return None

    def _find_body_by_name(self, name):

        """This subroutine returns a body from its name.

        Parameter
        ----------
        name : float, optional
            Name of a body.

        Returns
        -------
        BodyDB
        """

        for body in self._bodies:
            if body.mesh.name == name:
                return body

        print("warning : no body found with name %s" % name)
        return None

    def _initialize(self):

        """This subroutine updates and improve the hydrodynamic database (computation of RK and diffraction loads, impulse response functions, interpolation, etc.)
        """

        # Computing Froude-Krylov loads.
        self._hdb.froude_krylov_db

        # Storing diffraction loads.
        self._hdb.diffraction_db

        # Updating the wave directions.
        self._initialize_wave_dir()

        # Printing input data.
        self._discretization.initialize(self._hdb)

        # Impule response functions for radiation damping.
        tf = self.discretization.final_time
        dt = self.discretization.delta_time
        self._hdb.radiation_db.eval_impulse_response_function(tf=tf, dt=dt)

        # Infinite masses.
        self._hdb.radiation_db.eval_infinite_added_mass()

        # Impule response functions for advance speed.
        self._hdb.radiation_db.eval_impulse_response_function_Ku(tf=tf, dt=dt)

        # Interpolation of the diffraction loads with respect to the wave directions and the wave frequencies.
        self._interpolate_diffraction()

        # Interpolation of the Froude-Krylov loads with respect to the wave directions and the wave frequencies.
        self._interpolate_froude_krylov()

        #self._interpolate_radiation_damping()  # FIXME : ne marche pas ...

        for body in self._bodies:

            # Parameters for the new discretization of the HDB.
            body.discretization = self._discretization

            if body.wave_drift:
                body.wave_drift.discrete_wave_dir = self._discretization.wave_dirs * pi/ 180.
                body.wave_drift.initialize()

        self._is_initialized = True

        return

    def _initialize_wave_dir(self):

        """This subroutine updates the wave directions by adjusting the convention with the one used in FRyDoM, the FK and diffraction loads are updated accordingly.
        """

        # Vector of wave directions.
        self._hdb._wave_dirs = np.linspace(self._hdb.min_wave_dir, self._hdb.max_wave_dir, self._hdb.nb_wave_dir)

        fk_db = self._hdb.froude_krylov_db
        diff_db = self._hdb.diffraction_db

        # Symmetrize
        if self._hdb.min_wave_dir >= -np.float32() and self._hdb.max_wave_dir <= 180. + np.float32():
            self._hdb._wave_dirs, fk_db, diff_db = symetrize(self._hdb._wave_dirs, fk_db, diff_db)

        # Updating the FK and diffraction loads accordingly.
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

        # Sorting wave directions and creates the final FK and diffraction loads data.
        sort_dirs = np.argsort(self._hdb._wave_dirs)
        self._hdb._wave_dirs = self._hdb._wave_dirs[sort_dirs]
        self._hdb._froude_krylov_db.data = fk_db.data[:, :, sort_dirs]
        self._hdb._diffraction_db.data = diff_db.data[:, :, sort_dirs]

        self._hdb.min_wave_dir = np.min(self._hdb._wave_dirs)
        self._hdb.max_wave_dir = np.max(self._hdb._wave_dirs)
        self._hdb.nb_wave_dir = self._hdb._wave_dirs.shape[0]

        return

    def set_direction(self, d_angle, unit='deg'):

        """This subroutine sets the wave directions (still used?).

        Parameters
        ----------
        d_angle : float
            Angular step.
        unit : string, optional
            Unit of the angular step: 'deg' (by default) or 'rad'.
        """

        if unit == 'deg':
            d_angle *= pi/180.

        n = int(2.*pi/d_angle)
        d_angle = 2.*pi / float(n)

        self._wave_direction = np.linspace(0, 2.*pi, d_angle)

        return

    def set_frequencies(self, f_min, f_max, df, unit='rads'):

        """This subroutine sets the wave frequencies by wrapping the same subroutine of the DiscretizationDB class (still used?).

        Parameters
        ----------
        f_min : float
            Minimum frequency.
        f_max : float
            Maximum frequency.
        df : float
            Frequency step.
        unit : string, optional
            Unit of the frequency step: 'rads' (by default) or 'Hz'.
        """

        self._discretization.set_wave_frequencies(f_min, f_max, df, unit)

    def set_directions(self, min_angle, max_angle, delta_angle, unit='rad'):

        """This subroutine sets the wave directions by wrapping the same subroutine of the DiscretizationDB class (still used?).

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

        self._discretization.set_wave_direction(min_angle, max_angle, delta_angle, unit)

    def set_directions(self, delta_angle, unit='rad'):

        """This subroutine sets the wave directions between 0 and 2*pi by wrapping the same subroutine of the DiscretizationDB class (still used?).

        Parameters
        ----------
        delta_angle : float
            Wave direction angular step.
        unit : string, optional
            Unit of the angular step: 'rad' (by default) or 'deg'.
        """

        if unit == 'deg':
            delta_angle *= pi/180.

        self._discretization.set_wave_direction(0., 2.*pi, delta_angle, 'rad')

        return

    def _interpolate_diffraction(self):

        """This subroutine interpolates the diffractions loads with respect to the wave directions and the wave frequencies.
        """

        # Interpolation of the diffraction loads with respect to the wave directions.
        f_interp_dir = interpolate.interp1d(self._hdb.wave_dirs, self._hdb._diffraction_db.data, axis=2) # axis = 2 -> wave directions.
        self._hdb._diffraction_db.data = f_interp_dir(self._discretization.wave_dirs) # Application of the interpolation.

        # Interpolation of the diffraction loads with respect to the wave frequencies.
        f_interp_freq = interpolate.interp1d(self._hdb.omega, self._hdb._diffraction_db.data, axis=1) # axis = 1 -> wave frequencies.
        self._hdb._diffraction_db.data = f_interp_freq(self._discretization.wave_frequencies)

        return

    def _interpolate_froude_krylov(self):

        """This subroutine interpolates the Froude-Krylov loads with respect to the wave directions and the wave frequencies.
        """

        # Interpolation of the Froude-Krylov loads with respect to the wave directions.
        f_interp_dir = interpolate.interp1d(self._hdb.wave_dirs, self._hdb.froude_krylov_db.data, axis=2) # axis = 2 -> wave directions.
        self._hdb.froude_krylov_db.data = f_interp_dir(self._discretization.wave_dirs)

        # Interpolation of the Froude-Krylov loads with respect to the wave frequencies.
        f_interp_freq = interpolate.interp1d(self._hdb.omega, self._hdb.froude_krylov_db.data, axis=1) # axis = 1 -> wave frequencies.
        self._hdb.froude_krylov_db.data = f_interp_freq(self._discretization.wave_frequencies)

        return

    def _interpolate_radiation_damping(self):

        """This subroutine interpolates the radiation damping with respect to the wave directions and the wave frequencies (not used?).
        """

        #self._hdb.radiation_db._min_frequency = self.discretization.min_frequency
        #self._hdb.radiation_db._max_frequency = self.discretization.max_frequency
        #self._hdb.radiation_db._nb_frequencies = self.discretization.nb_frequencies

        print("nb frequencies : %i" % self._discretization.wave_frequencies.size)

        print(" shape ca (before) : ", self._hdb.radiation_db._ca.shape)

        f_interp_freq = interpolate.interp1d(self._hdb.omega, self._hdb.radiation_db._ca, axis=1)
        self._hdb._radiation_db._ca = f_interp_freq(self._discretization.wave_frequencies)

        print(" shape ca (after) : ", self._hdb.radiation_db._ca.shape)

        f_interp_freq = interpolate.interp1d(self._hdb.omega, self._hdb.radiation_db._cm, axis=1)
        self._hdb._radiation_db._cm = f_interp_freq(self._discretization.wave_frequencies)

        return


    def write_hdb5(self, output_file=None):

        """This subroutine writes the hydrodynamic database into a *.hdb5 file.

        Parameters
        ----------
        output_file : string, optional
            Name of the hdf5 output file.
        """

        if not self._is_initialized:

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
            # Writing all the data from _environment, _discretization and body data structures.
            with h5py.File(hdb5_file, 'w') as writer:
                self._environment.write_hdb5(writer)
                self._discretization.write_hdb5(writer)
                for body in self._bodies:
                    body.write_hdb5(writer)

        except IOError:
            raise IOError('Problem in writing HDB5 file at location %s' % hdb5_file)

        print('-------> "%s" has been written' % hdb5_file)








