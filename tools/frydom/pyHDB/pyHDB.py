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

import h5py
import numpy as np
from scipy import interpolate
from datetime import datetime

from wave_dispersion_relation_v2 import solve_wave_dispersion_relation
from wave_drift_db_v2 import WaveDriftDB

inf = float('inf') # Definition of infinity for depth.

class pyHDB():
    """
        Class for storing the hydrodynamique database.
    """

    def __init__(self):

        # Environmental data.
        self.rho_water = 0
        self.grav = 0
        self.depth = -1
        self.x_wave_measure = 0.
        self.y_wave_measure = 0.

        # Wave frequencies.
        self.nb_wave_freq = 0
        self.min_wave_freq = 0.
        self.max_wave_freq = 0.
        self.wave_freq = np.array([])
        self._iwcut = None

        # Wave directions.
        self.nb_wave_dir = 0
        self.min_wave_dir = 0.
        self.max_wave_dir = 0.
        self.wave_dir = np.array([])

        # Kochin parameters.
        self.has_kochin = False
        self.nb_angle_kochin = 0
        self.min_angle_kochin = 0.
        self.max_angle_kochin = 0.
        self.angle_kochin = np.array([])
        self.kochin_diffraction = None # Diffraction Kochin functions.
        self.kochin_radiation = None # Radiation Kochin functions.
        self.nb_dir_kochin = 0 # Different from self.nb_wave_dir if the symmetry was used.
        self.min_dir_kochin = 0. # Different from self.min_wave_dir if the symmetry was used.
        self.max_dir_kochin = 0. # Different from self.max_wave_dir if the symmetry was used.
        self.wave_dir_kochin = np.array([]) # Different from self.wave_dir if the symmetry was used.

        # Bodies.
        self.nb_bodies = 0
        self.bodies = []

        # Version.
        self.version = 2.0

        # Froude-Krylov loads.
        self._has_froude_krylov = False

        # Wave drift coeffcients (given in input of Nemoh2HDB).
        self._wave_drift = None

        # IRF.
        self.dt = None
        self.time = None
        self.nb_time_samples = None

        # Normalization length.
        self.normalization_length = 1.

        # RAO.
        self.has_RAO = False

        # Drift loads from Kochin functions.
        self.has_Drift_Kochin = False
        self.Wave_drift_force = None

    def set_wave_frequencies(self):
        """Frequency array of BEM computations in rad/s.

        Returns
        -------
        np.ndarray
        """

        self.wave_freq = np.linspace(self.min_wave_freq, self.max_wave_freq, self.nb_wave_freq, dtype=np.float)

    @property
    def omega(self):
        """Frequency array of BEM computations in rad/s

        Returns
        -------
        np.ndarray
        """
        if self._iwcut is None:
            return self.wave_freq
        else:
            return self.wave_freq[:self._iwcut]

    def get_full_omega(self):
        return self.wave_freq

    @property
    def wcut(self):

        """This function gives the cutting wave frequency.

        Returns
        -------
        float
            Cutting wave frequency.
        """

        if self._iwcut is None:
            return None
        else:
            w = self.get_full_omega()
            return w[self._iwcut]

    @wcut.setter
    def wcut(self, wcut):  # TODO: finir l'implementation

        """This function sets the cutting wave frequency.

        Parameter
        ----------
        wcut : float
            Cutting wave frequency.
        """

        if wcut is None:
            self._iwcut = None
        else:
            assert self._min_frequency < wcut <= self._max_frequency
            w = self.get_full_omega()
            self._iwcut = np.where(w >= wcut)[0][0]  # TODO: a verifier

    def set_wave_directions(self):
        """Frequency array of BEM computations in rad/s

        Returns
        -------
        np.ndarray
        """

        self.wave_dir = np.radians(np.linspace(self.min_wave_dir, self.max_wave_dir, self.nb_wave_dir, dtype=np.float))

    def set_wave_directions_Kochin(self):
        """This function initializes the incident wave directions in the diffraction kochin problems.

        Returns
        -------
        np.ndarray
        """

        self.nb_dir_kochin = self.nb_wave_dir
        self.min_dir_kochin = self.min_wave_dir
        self.max_dir_kochin = self.max_wave_dir
        self.wave_dir_kochin = np.radians(np.linspace(self.min_dir_kochin, self.max_dir_kochin, self.nb_dir_kochin, dtype=np.float))

    @property
    def wave_dir(self):
        """Wave direction angles array of BEM computations in radians

        Returns
        -------
        np.ndarray
            angles array in radians.
        """
        return self.wave_dir

    def append(self, body):

        """This function adds a body.

        Parameter
        ----------
        BodyDB : body
            Hydrodynamic body.
        """

        self.bodies.append(body)

    @property
    def nb_forces(self):

        """ This function computes the sum of the force modes for all bodies.
        Returns
        -------
        int
            Number of force modes for all bodies.
        """

        n = 0
        for body in self.bodies:
            n = n + int(body.Force_mask.sum())

        return n

    @property
    def nb_motions(self):

        """ This function computes the sum of the motion modes for all bodies.
        Returns
        -------
        int
            Number of motion modes for all bodies.
        """

        n = 0
        for body in self.bodies:
            n = n + int(body.Motion_mask.sum())

        return n

    @property
    def wave_drift(self):

        """This function gives the wave drift data of the body.

        Returns
        -------
        WaveDriftDB
            Wave drift data of the body.
        """

        return self._wave_drift

    def Eval_Froude_Krylov_loads(self):

        """ This functions computes the Froude-Krylov loads."""

        if not self._has_froude_krylov:

            """Computes the Froude-Krylov complex coefficients from indident wave field."""

            # Wave numbers.
            k_wave = solve_wave_dispersion_relation(self.wave_freq, self.depth, self.grav)

            # Computation of the Froude-Krylov loads for each body.
            for body in self.bodies:

                # Center of every face.
                centers = body.mesh.faces_centers
                x = centers[:, 0]
                y = centers[:, 1]
                z = centers[:, 2]

                # COMPUTING FIELDS.
                ctheta = np.cos(self.wave_dir) # cos(beta).
                stheta = np.sin(self.wave_dir) # sin(beta).

                kctheta = np.einsum('i, j -> ij', k_wave, ctheta) # k*cos(beta).
                kstheta = np.einsum('i, j -> ij', k_wave, stheta) # k*sin(beta).

                kw_bar = np.einsum('i, jk -> ijk', x - self.x_wave_measure, kctheta) # kw = k * (x - xref)*cos(beta).
                kw_bar += np.einsum('i, jk -> ijk', y - self.y_wave_measure, kstheta) # kw = k * ((x - xref)*cos(beta) + (y - yref)*sin(beta)).
                exp_jkw_bar = np.exp(1j * kw_bar) # e^(j * k * ((x - xref)*cos(beta) + (y - yref)*sin(beta))).

                if np.isinf(self.depth): # Infinite depth.

                    kxzph = np.einsum('i, j -> ij', z, k_wave) # k*z.
                    cih = np.exp(kxzph) # e^(kz).

                else: # Finite depth.

                    kxzph = np.einsum('i, j -> ij', z + depth, k_wave) # k*(z+h).
                    chkh_1 = 1. / np.cosh(k_wave * self.depth) # 1/ch(k*h).

                    cih = np.einsum('ij, j -> ij', np.cosh(kxzph), chkh_1) # ch(k(z+h)) / ch(k*h).

                cih_exp_jkw_bar = np.einsum('ij, ijk -> ijk', cih, exp_jkw_bar) # ch(k(z+h)) / ch(k*h) *  e^(j * k * ((x - xref)*cos(beta) + (y - yref)*sin(beta))).

                # Pressure.
                pressure = self.rho_water * self.grav * cih_exp_jkw_bar # rho * g * ch(k(z+h)) / ch(k*h) *  e^(j * k * ((x - xref)*cos(beta) + (y - yref)*sin(beta))).

                # Integration of the pressure of the wetted surface.
                for i_force in range(0,6):
                    if(body.Force_mask[i_force]):
                        nds = body.get_nds(i_force) # n*ds.
                        body.Froude_Krylov[i_force, :, :] = np.einsum('ijk, i -> jk', pressure, -nds) # Il s'agit de la normale entrante.

    def eval_impulse_response_function(self, tf = 30., dt = None, full=True):
        """Computes the impulse response functions.

        It uses the Ogilvie formulas based on radiation damping integration (Inverse Fourier Transform).

        Parameters
        ----------
        tf : float, optional
            Final time (seconds). Default is 30.
        dt : float, optional
            Time step (seconds). Default is None. If None, a time step is computed according to the max
            frequency of
            hydrodynamic coefficients (the Nyquist frequency is taken)
        full : bool, optional
            If True (default), it will use the full wave frequency range for computations.
        """

        # Time.
        if dt is None:
            # Using Shannon theorem.
            dt = pi / (10 * self.max_frequency)
        time = np.arange(start=0., stop=tf + dt, step=dt)
        tf = time[-1]  # It is overwriten !!

        # Storage.
        self.dt = dt
        self.time = time
        self.nb_time_samples = int(tf / self.dt) + 1

        # Wave frequency range.
        if full:
            w = self.get_full_omega()
        else:
            w = self.omega

        # Computation.
        wt = np.einsum('i, j -> ij', w, time)  # w*t.
        cwt = np.cos(wt)  # cos(w*t).

        for body in self.bodies:

            irf_data = np.empty(0, dtype=np.float)

            if full:
                ca = np.einsum('ijk, ij -> ijk', body.Damping, body._flags) # Damping.
            else:
                ca = body.radiation_damping(self._iwcut) # Damping.

            kernel = np.einsum('ijk, kl -> ijkl', ca, cwt)  # Damping*cos(wt).

            irf_data = (2 / np.pi) * np.trapz(kernel, x=w, axis=2)  # Int(Damping*cos(wt)*dw).

            body.irf = irf_data

    def eval_infinite_added_mass(self, full=True):
        """Evaluates the infinite added mass matrix coefficients using Ogilvie formula.

        Parameter
        ---------
        full : bool, optional
            If True (default), it will use the full frequency range for computations.

         It uses the Ogilvie formula to get the coefficients from the impulse response functions.
        """

        # Time.
        dt = self.dt
        time = self.time
        tf = time[-1]

        # Wave frequency range.
        if full:
            w = self.get_full_omega()
        else:
            w = self.omega

        # Computation.
        wt = np.einsum('i, j -> ij', w, time)  # w*t.
        sin_wt = np.sin(wt) # sin(w*t).

        for body in self.bodies:

            # Initialization.
            body.Inf_Added_mass = np.zeros((6, 6*self.nb_bodies), dtype = np.float)

            # IRF.
            irf = body.irf

            # Added mass.
            if full:
                cm = body.Added_mass
            else:
                cm = body.radiation_added_mass(self._iwcut)

            kernel = np.einsum('ijk, lk -> ijlk', irf, sin_wt)  # irf*sin(w*t).
            integral = np.einsum('ijk, k -> ijk', np.trapz(kernel, x=time, axis=3), 1. / w)  # 1/w * int(irf*sin(w*t),dt).

            body.Inf_Added_mass = (cm + integral).mean(axis=2)  # mean( A(w) + 1/w * int(irf*sin(w*t),dt) ) wrt w.

    def eval_impulse_response_function_Ku(self, full=True):
        """Computes the impulse response functions relative to the ship advance speed

        ref : F. Rongère et al. (Journées de l'Hydrodynamique 2010 - Nantes).

        Parameter
        ---------
        full : bool, optional
            If True (default), it will use the full frequency range for computations.
        """

        # Time.
        dt = self.dt
        time = self.time
        tf = time[-1]

        # Wave frequency range.
        if full:
            w = self.get_full_omega()
        else:
            w = self.omega

        # Computation.
        wt = np.einsum('i, j ->ij', w, time)  # w*t.
        cwt = np.cos(wt)  # cos(w*t).

        for body in self.bodies:

            # Initialization.
            irf_data = np.empty(0, dtype=np.float)

            if full:
                cm = np.einsum('ijk, ij -> ijk', body.Added_mass, body._flags) # Added mass.
            else:
                cm = self.radiation_added_mass(self._iwcut) # Added mass.

            cm_inf = body.Inf_Added_mass

            cm_diff = np.zeros(cm.shape)
            for j in range(w.size):
                cm_diff[:, :, j] = cm_inf[:, :] - cm[:, :, j] # A(inf) - A(w).

            cm_diff[:, 4, :] = -cm_diff[:, 2, :]
            cm_diff[:, 5, :] = cm_diff[:, 1, :]
            cm_diff[:, 0, :] = 0.
            cm_diff[:, 1, :] = 0.
            cm_diff[:, 2, :] = 0.
            cm_diff[:, 3, :] = 0.

            kernel = np.einsum('ijk, kl -> ijkl', cm_diff, cwt) # int((A(inf) - A(w))*L*cos(wt),dw).

            irf_data = (2. / np.pi) * np.trapz(kernel, x=w, axis=2) # (2/pi) * int((A(inf) - A(w))*L*cos(wt),dw).

            body.irf_ku = irf_data

    def interpolation(self,discretization):
        """this function interpolates with respect to the wave directions and the wave frequencies."""

        for body in self.bodies:

            # Diffraction loads - Wave directions.
            f_interp_diffraction_dir = interpolate.interp1d(self.wave_dir, body.Diffraction, axis=2)  # axis = 2 -> wave directions.
            body.Diffraction = f_interp_diffraction_dir(discretization._wave_dirs)  # Application of the interpolation.

            # Diffraction loads - Wave frequencies.
            f_interp_diffraction_freq = interpolate.interp1d(self.wave_freq, body.Diffraction, axis=1) # axis = 1 -> wave frequencies.
            body.Diffraction = f_interp_diffraction_freq(discretization._wave_frequencies)

            # Froude-Krylov loads - Wave directions.
            f_interp_fk_dir = interpolate.interp1d(self.wave_dir, body.Froude_Krylov, axis=2)  # axis = 2 -> wave directions.
            body.Froude_Krylov = f_interp_fk_dir(discretization._wave_dirs)  # Application of the interpolation.

            # Froude-Krylov loads - Wave frequencies.
            f_interp_fk_freq = interpolate.interp1d(self.wave_freq, body.Froude_Krylov, axis=1) # axis = 1 -> wave frequencies.
            body.Froude_Krylov = f_interp_fk_freq(discretization._wave_frequencies)

            # Added mass - Wave frequencies.
            f_interp_Added_mass_freq = interpolate.interp1d(self.wave_freq, body.Added_mass, axis=2) # axis = 2 -> wave frequencies.
            body.Added_mass = f_interp_Added_mass_freq(discretization._wave_frequencies)

            # Damping - Wave frequencies.
            f_interp_Damping_freq = interpolate.interp1d(self.wave_freq, body.Damping, axis=2)  # axis = 2 -> wave frequencies.
            body.Damping = f_interp_Damping_freq(discretization._wave_frequencies)

        # Kochin functions.
        if(self.has_kochin):

            # Diffraction Kochin functions - Wave directions.
            f_interp_kochin_diffraction_dir = interpolate.interp1d(self.wave_dir, self.kochin_diffraction, axis=0)  # axis = 0 -> wave directions.
            self.kochin_diffraction = f_interp_kochin_diffraction_dir(discretization._wave_dirs)  # Application of the interpolation.

            # Diffraction Kochin functions - Wave frequencies.
            f_interp_kochin_diffraction_freq = interpolate.interp1d(self.wave_freq, self.kochin_diffraction, axis=1)  # axis = 1 -> wave frequencies.
            self.kochin_diffraction = f_interp_kochin_diffraction_freq(discretization._wave_frequencies)  # Application of the interpolation.

            # Radiation Kochin functions - Wave frequencies.
            f_interp_kochin_radiation_freq = interpolate.interp1d(self.wave_freq, self.kochin_radiation, axis=1)  # axis = 1 -> wave frequencies.
            self.kochin_radiation = f_interp_kochin_radiation_freq(discretization._wave_frequencies)  # Application of the interpolation.

        # Update wave directions and frequencies vectors.
        self.min_wave_freq = discretization._min_frequency
        self.max_wave_freq = discretization._max_frequency
        self.nb_wave_freq = discretization.nb_frequencies
        self.wave_freq = discretization._wave_frequencies

        self.min_wave_dir = discretization._min_angle
        self.max_wave_dir = discretization._max_angle
        self.nb_wave_dir = discretization._nb_wave_directions
        self.wave_dir = discretization._wave_dirs

        if (self.has_kochin):
            self.min_dir_kochin = discretization._min_angle
            self.max_dir_kochin = discretization._max_angle
            self.nb_dir_kochin = discretization._nb_wave_directions
            self.wave_dir_kochin = discretization._wave_dirs

    def symetrize(self):

        """This function updates the hdb due to a modification of the wave direction convention."""

        ndir = self.nb_wave_dir
        nw = self.nb_wave_freq

        for i in range(ndir):

            if(np.degrees(self.wave_dir[i]) > np.float32(0.)):

                # New wave direction.
                new_dir = -np.degrees(self.wave_dir[i]) % 360
                if new_dir < 0:
                    new_dir += 360.

                # Add corresponding data.
                self.wave_dir = np.append(self.wave_dir, np.radians(new_dir))

                # Loop over the bodies.
                for body in self.bodies:

                    # Froude-Krylov loads.
                    fk_db_temp = np.copy(body.Froude_Krylov[:, :, i])
                    fk_db_temp[(1, 3, 5), :] = -fk_db_temp[(1, 3, 5), :] # 1 = sway, 3 = roll and 5 = yaw.
                    body.Froude_Krylov = np.concatenate((body.Froude_Krylov, fk_db_temp.reshape(6, nw, 1)), axis=2) # Axis of the wave directions.

                    # Diffraction loads.
                    diff_db_temp = np.copy(body.Diffraction[:, :, i])
                    diff_db_temp[(1, 3, 5), :] = -diff_db_temp[(1, 3, 5), :] # 1 = sway, 3 = roll and 5 = yaw.
                    body.Diffraction = np.concatenate((body.Diffraction, diff_db_temp.reshape(6, nw, 1)), axis=2) # Axis of the wave directions.

                    # RAO.
                    if(self.has_RAO):
                        RAO_db_temp = np.copy(body.RAO[:, :, i])
                        RAO_db_temp[(1, 3, 5), :] = -RAO_db_temp[(1, 3, 5), :]  # 1 = sway, 3 = roll and 5 = yaw.
                        body.RAO = np.concatenate((body.RAO, RAO_db_temp.reshape(6, nw, 1)), axis=2)  # Axis of the wave directions.

                # Wave drift loads.
                if(self.has_Drift_Kochin):
                    Drift_db_temp = np.copy(self.Wave_drift_force[:, :, i])
                    Drift_db_temp[(1, 2), :] = -Drift_db_temp[(1, 2), :] # 1 = sway and 2 = yaw.
                    self.Wave_drift_force = np.concatenate((self.Wave_drift_force, Drift_db_temp.reshape(3, nw, 1)), axis=2)  # Axis of the wave directions.

    def _initialize_wave_dir(self):

        """This function updates the wave directions by adjusting the convention with the one used in FRyDoM, the FK and diffraction loads are updated accordingly."""

        # Symmetrization.
        if self.min_wave_dir >= -np.float32() and self.max_wave_dir <= 180. + np.float32():
            self.symetrize()

        # Updating the loads accordingly.
        n180 = 0
        i360 = -9
        for idir in range(self.wave_dir.size):
            wave_dir = self.wave_dir[idir]

            if abs(np.degrees(wave_dir)) < 0.01:
                i360 = idir
            elif abs(np.degrees(wave_dir) - 180) < 0.01:
                n180 += 1
                if n180 == 2:
                    self.wave_dir[idir] = np.radians(360.)

                    # Loop over the bodies.
                    for body in self.bodies:

                        # Froude-Krylov loads.
                        body.Froude_Krylov[:, :, idir] = body.Froude_Krylov[:, :, i360]

                        # Diffraction loads.
                        body.Diffraction[:, :, idir] = body.Diffraction[:, :, i360]

                        # RAO.
                        if (self.has_RAO):
                            body.RAO[:, :, idir] = body.RAO[:, :, i360]

                    # Wave drift loads.
                    if (self.has_Drift_Kochin):
                        self.Wave_drift_force[:, :, idir] = self.Wave_drift_force[:, :, i360]

        # Sorting wave directions and creates the final FK and diffraction loads data.
        sort_dirs = np.argsort(self.wave_dir)
        self.wave_dir = self.wave_dir[sort_dirs]

        # Loop over the bodies.
        for body in self.bodies:

            # Froude-Krylov loads.
            body.Froude_Krylov = body.Froude_Krylov[:, :, sort_dirs]

            # Diffraction loads.
            body.Diffraction = body.Diffraction[:, :, sort_dirs]

            # RAO.
            if (self.has_RAO):
                body.RAO = body.RAO[:, :, sort_dirs]

        # Wave drift loads.
        if (self.has_Drift_Kochin):
            self.Wave_drift_force = self.Wave_drift_force[:, :, sort_dirs]

        # Update parameters.
        self.min_wave_dir = np.min(self.wave_dir)
        self.max_wave_dir = np.max(self.wave_dir)
        self.nb_wave_dir = self.wave_dir.shape[0]

    def write_hdb5(self, hdb5_file):
        """This function writes the hydrodynamic database into a *.hdb5 file.

        Parameter
        ---------
        output_file : string, optional.
            Name of the hdf5 output file.
        """

        with h5py.File(hdb5_file, 'w') as writer:

            # Environment.
            self.write_environment(writer)

            # Discretization.
            self.write_discretization(writer)

            # Bodies.
            for body in self.bodies:
                self.write_body(writer, body)

            # Wave drift coefficients.
            if (self._wave_drift and self.has_Drift_Kochin is False):
                self.write_wave_drift(writer, "/WaveDrift")
            elif(self.has_Drift_Kochin):
                self.write_wave_drift_Kochin(writer, "/WaveDrift")

            # Version.
            self.write_version(writer)

    def write_environment(self, writer):
        """This function writes the environmental data into the *.hdb5 file.

        Parameter
        ---------
        writer : string.
            *.hdb5 file.
        """

        # Date.
        dset = writer.create_dataset('CreationDate', data=str(datetime.now()))
        dset.attrs['Description'] = "Date of the creation of this database."

        # Gravity acceleration.
        dset = writer.create_dataset('GravityAcc', data=self.grav)
        dset.attrs['Unit'] = 'm/s**2'
        dset.attrs['Description'] = "Gravity acceleration."

        # Water density.
        dset = writer.create_dataset('WaterDensity', data=self.rho_water)
        dset.attrs['Unit'] = 'kg/m**3'
        dset.attrs['Description'] = 'Water Density.'

        # Normalisation length.
        dset = writer.create_dataset('NormalizationLength', data=self.normalization_length)
        dset.attrs['Unit'] = 'm'
        dset.attrs['Description'] = 'Normalization length.'

        # Water depth.
        dset = writer.create_dataset('WaterDepth', data=self.depth)
        dset.attrs['Unit'] = 'm'
        dset.attrs['Description'] = 'Water depth: 0 for infinite depth and positive for finite depth.'

        # Number of bodies.
        dset = writer.create_dataset('NbBody', data=self.nb_bodies)
        dset.attrs['Description'] = 'Number of hydrodynamic bodies.'

    def write_discretization(self,writer):
        """This function writes the discretization parameters into the *.hdb5 file.

        Parameter
        ---------
        Writer : string.
            *.hdb5 file.
        """

        discretization_path = "/Discretizations"
        writer.create_group(discretization_path)

        # Frequency discretization.

        frequential_path = discretization_path + "/Frequency"

        dset = writer.create_dataset(frequential_path + "/NbFrequencies", data=self.nb_wave_freq)
        dset.attrs['Description'] = "Number of frequencies"

        dset = writer.create_dataset(frequential_path + "/MinFrequency", data=self.min_wave_freq)
        dset.attrs['Unit'] = "rad/s"
        dset.attrs['Description'] = "Minimum frequency."

        dset = writer.create_dataset(frequential_path + "/MaxFrequency", data=self.max_wave_freq)
        dset.attrs['Unit'] = "rad/s"
        dset.attrs['Description'] = "Maximum frequency."

        # Wave direction discretization.

        wave_direction_path = discretization_path + "/WaveDirections"

        dset = writer.create_dataset(wave_direction_path + "/NbWaveDirections", data=self.nb_wave_dir)
        dset.attrs['Description'] = "Number of wave directions."

        dset = writer.create_dataset(wave_direction_path + "/MinAngle", data=np.degrees(self.min_wave_dir))
        dset.attrs['Unit'] = "deg"
        dset.attrs['Description'] = "Minimum wave direction."

        dset = writer.create_dataset(wave_direction_path + "/MaxAngle", data=np.degrees(self.max_wave_dir))
        dset.attrs['Unit'] = "deg"
        dset.attrs['Description'] = "Maximum wave direction."

        # Time sample.

        time_path = discretization_path + "/Time"

        dset = writer.create_dataset(time_path + "/NbTimeSample", data=self.nb_time_samples)
        dset.attrs['Description'] = "Number of time samples."

        dset = writer.create_dataset(time_path + "/FinalTime", data=self.time[-1])
        dset.attrs['Unit'] = "s"
        dset.attrs['Description'] = "Final time for the evaluation of the impulse response functions."

        dset = writer.create_dataset(time_path + "/TimeStep", data=self.dt)
        dset.attrs['Unit'] = "s"
        dset.attrs['Description'] = "Time step."

    def write_mode(self, writer, body, ForceOrMotion, body_modes_path="/Modes"):
        """This function writes the force and motion modes into the *.hdb5 file.

        Parameters
        ----------
        Writer : string
            *.hdb5 file.
        body : BodyDB.
            Body.
        ForceOrMotion : int
            0 for Force, 1 for Motion.
        body_modes_path : string, optional
            Path to body modes.
        """

        if(ForceOrMotion == 0): # Force.
            dset = writer.create_dataset(body_modes_path + "/NbForceModes", data=6)
            dset.attrs['Description'] = "Number of force modes for body number %u" % body.i_body
        else: # Motion.
            dset = writer.create_dataset(body_modes_path + "/NbMotionModes", data=6)
            dset.attrs['Description'] = "Number of motion modes for body number %u" % body.i_body

        for iforce in range(0,6):

            if(ForceOrMotion == 0): # Force.
                mode_path = body_modes_path + "/ForceModes/Mode_%u" % iforce
            else: # Motion.
                mode_path = body_modes_path + "/MotionModes/Mode_%u" % iforce
            writer.create_group(mode_path)
            if(iforce == 0 or iforce == 3):
                direction = np.array([1., 0., 0.])
            elif(iforce == 1 or iforce == 4):
                direction = np.array([0., 1., 0.])
            elif(iforce == 2 or iforce == 5):
                direction = np.array([0., 0., 1.])
            writer.create_dataset(mode_path + "/Direction", data=direction)

            if (iforce <= 2):
                writer.create_dataset(mode_path + "/Type", data='LINEAR')
            elif (iforce >= 3):
                writer.create_dataset(mode_path + "/Type", data='ANGULAR')
                writer.create_dataset(mode_path + "/Point", data=body.point)

    def write_mask(self, writer, body, mask_path="/Mask"):
        """This function writes the Force and Motion masks into the *.hdb5 file.

        Parameters
        ----------
        Writer : string
            *.hdb5 file.
        body : BodyDB.
            Body.
        ForceOrMotion : int
            0 for Force, 1 for Motion.
        body_modes_path : string, optional
            Path to body modes.
        """

        writer.create_group(mask_path)
        writer.create_dataset(mask_path + "/MotionMask", data=body.Motion_mask.astype(int))
        writer.create_dataset(mask_path + "/ForceMask", data=body.Force_mask.astype(int))

    def write_mesh(self, writer, body, mesh_path="/Mesh"):

        """This function writes the mesh quantities into the *.hdb5 file.

        Parameters
        ----------
        Writer : string
            *.hdb5 file.
        body : BodyDB.
            Body.
        mesh_path : string
            Path to the mesh folder.
        """

        writer.create_dataset(mesh_path + "/NbVertices", data=body.mesh.nb_vertices)
        writer.create_dataset(mesh_path + "/Vertices", data=body.mesh.vertices)
        writer.create_dataset(mesh_path + "/NbFaces", data=body.mesh.nb_faces)
        writer.create_dataset(mesh_path + "/Faces", data=body.mesh.faces)

    def write_excitation(self, writer, body, excitation_path="/Excitation"):

        """This function writes the diffraction and Froude-Krylov loads into the *.hdb5 file.

        Parameters
        ----------
        Writer : string
            *.hdb5 file.
        body : BodyDB.
            Body.
        excitation_path : string, optional
            Path to excitation loads.
        """

        # Froude-Krylov loads.

        fk_path = excitation_path + "/FroudeKrylov"
        writer.create_group(fk_path)

        for idir in range(0,self.nb_wave_dir):

            wave_dir_path = fk_path + "/Angle_%u" % idir
            writer.create_group(wave_dir_path)

            dset = writer.create_dataset(wave_dir_path + "/Angle", data=np.degrees(self.wave_dir[idir]))
            dset.attrs['Unit'] = 'deg'
            dset.attrs['Description'] = "Wave direction."

            # Real parts.
            dset = writer.create_dataset(wave_dir_path + "/RealCoeffs", data=body.Froude_Krylov[:, :, idir].real)
            dset.attrs['Unit'] = ''
            dset.attrs['Description'] = "Real part of the Froude-Krylov loads " \
                                        "on body %u for a wave direction of %.1f deg." % \
                                        (body.i_body, np.degrees(self.wave_dir[idir]))

            # Imaginary parts.
            dset = writer.create_dataset(wave_dir_path + "/ImagCoeffs", data=body.Froude_Krylov[:, :, idir].imag)
            dset.attrs['Unit'] = ''
            dset.attrs['Description'] = "Imaginary part of the Froude-Krylov loads " \
                                        "on body %u for a wave direction of %.1f deg." % \
                                        (body.i_body, np.degrees(self.wave_dir[idir]))

        # Diffraction loads.

        diffraction_path = excitation_path + "/Diffraction"
        writer.create_group(diffraction_path)

        for idir in range(0,self.nb_wave_dir):

            wave_dir_path = diffraction_path + "/Angle_%u" % idir
            writer.create_group(wave_dir_path)

            dset = writer.create_dataset(wave_dir_path + "/Angle", data=np.degrees(self.wave_dir[idir]))
            dset.attrs['Unit'] = 'deg'
            dset.attrs['Description'] = "Wave direction."

            # Real parts.
            writer.create_dataset(wave_dir_path + "/RealCoeffs", data=body.Diffraction[:, :, idir].real)
            dset.attrs['Unit'] = ''
            dset.attrs['Description'] = "Real part of the diffraction loads " \
                                        "on body %u for a wave direction of %.1f deg." % \
                                        (body.i_body, np.degrees(self.wave_dir[idir]))

            # Imaginary parts.
            writer.create_dataset(wave_dir_path + "/ImagCoeffs", data=body.Diffraction[:, :, idir].imag)
            dset.attrs['Unit'] = ''
            dset.attrs['Description'] = "Imaginary part of the diffraction loads " \
                                        "on body %u for a wave direction of %.1f deg." % \
                                        (body.i_body, np.degrees(self.wave_dir[idir]))

    def write_radiation(self, writer, body, radiation_path="/Radiation"):

        """This function writes the added mass and damping coefficients and the impulse response functions with and without forward speed into the *.hdb5 file.

        Parameters
        ----------
        Writer : string
            *.hdb5 file.
        body : BodyDB.
            Body.
        radiation_path : string, optional
            Path to radiation loads.
        """

        writer.create_group(radiation_path)

        for j in range(self.nb_bodies):

            radiation_body_motion_path = radiation_path + "/BodyMotion_%u" % j

            dg = writer.create_group(radiation_body_motion_path)
            dg.attrs['Description'] = "Hydrodynamic coefficients for motion of body %u that radiates waves and " \
                                      " generate force on body %u." % (j, body.i_body)

            added_mass_path = radiation_body_motion_path + "/AddedMass"
            dg = writer.create_group(added_mass_path)
            dg.attrs['Description'] = "Added mass coefficients for acceleration of body %u that radiates waves " \
                                      "and generates forces on body %u." % (j, body.i_body)

            radiation_damping_path = radiation_body_motion_path + "/RadiationDamping"
            dg = writer.create_group(radiation_damping_path)
            dg.attrs['Description'] = "Damping coefficients for velocity of body %u that radiates waves " \
                                      "and generates forces on body %u." % (j, body.i_body)

            irf_path = radiation_body_motion_path + "/ImpulseResponseFunctionK"
            dg = writer.create_group(irf_path)
            dg.attrs['Description'] = "Impulse response functions K for velocity of body %u that radiates waves " \
                                      "and generates forces on body %u." % (j, body.i_body)

            irf_ku_path = radiation_body_motion_path + "/ImpulseResponseFunctionKU"
            dg = writer.create_group(irf_ku_path)
            dg.attrs['Description'] = "Impulse response functions Ku for velocity of body %u that radiates waves " \
                                      "and generates forces on body %u." % (j, body.i_body)

            dset = writer.create_dataset(radiation_body_motion_path + "/InfiniteAddedMass",
                                         data=body.Inf_Added_mass[:,6*j:6*(j+1)])
            dset.attrs['Description'] = "Infinite added mass matrix that modifies the apparent mass of body %u from " \
                                        "acceleration of body %u." % (body.i_body, j)

            for iforce in range(0,6):

                # Added mass.
                dset = writer.create_dataset(added_mass_path + "/DOF_%u" % iforce, data=body.Added_mass[:, 6*j+iforce, :])
                dset.attrs['Unit'] = ""
                dset.attrs['Description'] = "Added mass coefficients for an acceleration of body %u and force on " \
                                            "body %u." % (j, body.i_body)

                # Damping.
                dset = writer.create_dataset(radiation_damping_path + "/DOF_%u" % iforce,
                                        data=body.Damping[:, 6*j+iforce, :])
                dset.attrs['Unit'] = ""
                dset.attrs['Description'] = "Wave damping coefficients for an acceleration of body %u and force " \
                                            "on body %u." % (j, body.i_body)

                # Impulse response function without forward speed.
                dset = writer.create_dataset(irf_path + "/DOF_%u" % iforce,
                                        data=body.irf[:, 6*j+iforce, :])
                dset.attrs['Description'] = "Impulse response functions"

                # Impulse response function Ku
                dset = writer.create_dataset(irf_ku_path + "/DOF_%u" % iforce,
                                        data=body.irf_ku[:, 6*j+iforce, :])
                dset.attrs['Description'] = "Impulse response functions Ku"

    def write_hydrostatic(self, writer, body, hydrostatic_path="/Hydrostatic"):

        """This function writes the hydrostatic stiffness matrix into the *.hdb5 file.

        Parameters
        ----------
        Writer : string
            *.hdb5 file.
        body : BodyDB.
            Body.
        hydrostatic_path : string, optional
            Path to hydrostatic stiffness matrix.
        """

        dg = writer.create_group(hydrostatic_path)

        dset = dg.create_dataset(hydrostatic_path + "/StiffnessMatrix", data=body.hydrostatic.matrix)
        dset.attrs['Description'] = "Hydrostatic stiffness matrix."

    def write_mass_matrix(self, writer, body, inertia_path="/Inertia"):

        """This function writes the mass matrix matrix into the *.hdb5 file.

        Parameters
        ----------
        Writer : string
            *.hdb5 file.
        body : BodyDB.
            Body.
        inertia_path : string, optional
            Path to inertia matrix.
        """

        dg = writer.create_group(inertia_path)

        dset = dg.create_dataset(inertia_path + "/InertiaMatrix", data=body.inertia.matrix)
        dset.attrs['Description'] = "Mass matrix."

    def write_RAO(self, writer, body, RAO_path="/RAO"):

        """This function writes the RAO into the *.hdb5 file.

        Parameters
        ----------
        Writer : string
            *.hdb5 file.
        body : BodyDB.
            Body.
        excitation_path : string, optional
            Path to excitation loads.
        """

        writer.create_group(RAO_path)

        for idir in range(0,self.nb_wave_dir):

            wave_dir_path = RAO_path + "/Angle_%u" % idir
            writer.create_group(wave_dir_path)

            dset = writer.create_dataset(wave_dir_path + "/Angle", data=np.degrees(self.wave_dir[idir]))
            dset.attrs['Unit'] = 'deg'
            dset.attrs['Description'] = "Wave direction."

            # Amplitude.
            dset = writer.create_dataset(wave_dir_path + "/Amplitude", data=np.absolute(body.RAO[:, :, idir]))
            dset.attrs['Unit'] = ''
            dset.attrs['Description'] = "Amplitude of the RAO of" \
                                        " body %u for a wave direction of %.1f deg." % \
                                        (body.i_body, np.degrees(self.wave_dir[idir]))

            # Phase.
            dset = writer.create_dataset(wave_dir_path + "/Phase", data=np.angle(body.RAO[:, :, idir], deg=True))
            dset.attrs['Unit'] = ''
            dset.attrs['Description'] = "Phase of the RAO of" \
                                        " body %u for a wave direction of %.1f deg." % \
                                        (body.i_body, np.degrees(self.wave_dir[idir]))

    def write_body(self, writer, body):
        """This function writes the body data into the *.hdb5 file.

        Parameters
        ----------
        writer : string.
            *.hdb5 file.
        body : BodyDB.
            Body.
        """

        body_path = '/Bodies/Body_%u' % body.i_body
        dset = writer.create_group(body_path)

        # Index of the body.
        dset = writer.create_dataset(body_path + "/ID", data=body.i_body)
        dset.attrs['Description'] = "Body index"

        # Force modes.
        self.write_mode(writer, body, 0, body_path + "/Modes")

        # Motion modes.
        self.write_mode(writer, body, 1, body_path + "/Modes")

        # Masks.
        self.write_mask(writer, body, body_path + "/Mask")

        # Mesh file.
        self.write_mesh(writer, body, body_path + "/Mesh")

        # Diffraction and Froude-Krylov loads.
        self.write_excitation(writer, body, body_path + "/Excitation")

        # Added mass and damping coefficients.
        self.write_radiation(writer, body, body_path + "/Radiation")

        # Hydrostatics.
        if body._hydrostatic:
            self.write_hydrostatic(writer, body, body_path + "/Hydrostatic")

        # Mass matrix.
        if body._inertia:
            self.write_mass_matrix(writer, body, body_path + "/Inertia")

        # RAO.
        if(self.has_RAO):
            self.write_RAO(writer, body, body_path + "/RAO")

    def write_wave_drift(self, writer, wave_drift_path="/WaveDrift"):

        """This function writes the wave drift loads into the *.hdb5 file.

        Parameters
        ----------
        Writer : string
            *.hdb5 file.
        wave_drift_path : string, optional
            Path to wave drift loads.
        """

        dg = writer.create_group(wave_drift_path)

        for key, mode in self.wave_drift.modes.items():
            grp_modes = dg.require_group(mode.name)

            for i_angle, angle in enumerate(mode.heading):
                grp_dir = grp_modes.require_group("heading_%i" % i_angle)

                # Set heading angle.
                dset = grp_dir.create_dataset("heading", data=angle)
                dset.attrs['Unit'] = 'rad'
                dset.attrs['Description'] = "Heading angle"

                # Set data.
                dset = grp_dir.create_dataset("data", data=mode.data[i_angle, :])
                dset.attrs['Description'] = "Wave Drift force coefficients"

        # Set frequency.
        if(self.wave_drift.discrete_frequency is not None):
            dset = dg.create_dataset("freq", data=self.wave_drift.discrete_frequency)
            dset.attrs['Unit'] = "rads"
            dset.attrs['Description'] = "Time discretization of the data"

        # Set sym.
        dset = dg.create_dataset("sym_x", data=self.wave_drift.sym_x)
        dset.attrs['Description'] = "Symmetry along x"
        dset = dg.create_dataset('sym_y', data=self.wave_drift.sym_y)
        dset.attrs['Description'] = "Symmetry along y"

    def write_wave_drift_Kochin(self, writer, wave_drift_path="/WaveDrift"):

        """This functions writes the wave drift coefficients in the hdb5 ouput file."""

        # Folder "WaveDrift".
        wave_drift_path = '/WaveDrift'
        dg = writer.create_group(wave_drift_path)

        # Set frequency.
        dset = dg.create_dataset("freq", data=self.omega)
        dset.attrs['Unit'] = "rad/s"
        dset.attrs['Description'] = "Wave frequencies."

        # Set sym.
        dset = dg.create_dataset("sym_x", data=0)
        dset.attrs['Description'] = "Symmetry along x"
        dset = dg.create_dataset('sym_y', data=0)
        dset.attrs['Description'] = "Symmetry along y"

        ###################################################"
        # x-axis - Force.
        ###################################################"

        # Folder "WaveDrift\surge".
        grp_modes = dg.require_group("surge")

        # Loop over the wave directions.
        for ibeta in range(0, self.nb_wave_dir):

            # Folder "WaveDrift\surge\heading_i"
            grp_dir = grp_modes.require_group("heading_%i" % ibeta)

            # Set heading angle
            dset = grp_dir.create_dataset("heading", data=self.wave_dir[ibeta])
            dset.attrs['Unit'] = 'rad'
            dset.attrs['Description'] = "Incident wave directions."

            # Set data
            dset = grp_dir.create_dataset("data", data=self.Wave_drift_force[0, :, ibeta])
            dset.attrs['Description'] = "Wave drift force in surge."

        ###################################################"
        # y-axis - Force.
        ###################################################"

        # Folder "WaveDrift\surge".
        grp_modes = dg.require_group("sway")

        # Loop over the wave directions.
        for ibeta in range(0, self.nb_wave_dir):

            # Folder "WaveDrift\surge\heading_i"
            grp_dir = grp_modes.require_group("heading_%i" % ibeta)

            # Set heading angle
            dset = grp_dir.create_dataset("heading", data=self.wave_dir[ibeta])
            dset.attrs['Unit'] = 'rad'
            dset.attrs['Description'] = "Heading angle"

            # Set data
            dset = grp_dir.create_dataset("data", data=self.Wave_drift_force[1, :, ibeta])
            dset.attrs['Description'] = "Wave drift force in sway"

        ###################################################"
        # z-axis - Moment.
        ###################################################"

        # Folder "WaveDrift\surge".
        grp_modes = dg.require_group("yaw")

        # Loop over the wave directions.
        for ibeta in range(0, self.nb_wave_dir):

            # Folder "WaveDrift\surge\heading_i"
            grp_dir = grp_modes.require_group("heading_%i" % ibeta)

            # Set heading angle
            dset = grp_dir.create_dataset("heading", data=self.wave_dir[ibeta])
            dset.attrs['Unit'] = 'rad'
            dset.attrs['Description'] = "Heading angle"

            # Set data
            dset = grp_dir.create_dataset("data", data=self.Wave_drift_force[2, :, ibeta])
            dset.attrs['Description'] = "Wave drift moment in yaw."

    def write_version(self, writer):
        """This function writes the version of the *.hdb5 file.

        Parameter
        ---------
        Writer : string
            *.hdb5 file.
        """

        # Version.
        dset = writer.create_dataset('Version', data=self.version)
        dset.attrs['Description'] = "Version of the hdb5 output file."

