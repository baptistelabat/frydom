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

import numpy as np
from scipy import interpolate

from wave_dispersion_relation_v2 import solve_wave_dispersion_relation
from wave_drift_db_v2 import WaveDriftDB

inf = float('inf')  # Definition of infinity for depth

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

        # Wave drift.
        self._wave_drift = None

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

    def Eval_Froude_Krylov_loads(self):

        """ This functions computes the Froude-Krylov loads."""

        if not self._has_froude_krylov:

            """Computes the Froude-Krylov complex coefficients from indident wave field"""
            # TODO: sortir le potentiel, les pressions et les vitesses normales...

            # Wave numbers.
            k_wave = solve_wave_dispersion_relation(self.wave_freq, self.depth, self.grav)

            # Computation of the Froude-Krylov loads for each body.
            for body in self.bodies:

                # Center of every face.
                centers = body.mesh.faces_centers
                x = centers[:, 0]
                y = centers[:, 1]
                z = centers[:, 2]

                # COMPUTING FIELDS
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
                    nds = body.get_nds(i_force) # n*ds.
                    body.Froude_Krylov[i_force, :, :] = np.einsum('ijk, i -> jk', pressure, -nds) # Il s'agit de la normale entrante.

    def _initialize_wave_dir(self):

        """This function updates the wave directions by adjusting the convention with the one used in FRyDoM, the FK and diffraction loads are updated accordingly."""

        # Symmetrize
        if self.min_wave_dir >= -np.float32() and self.max_wave_dir <= 180. + np.float32():
            self.symetrize()

        # Updating the FK and diffraction loads accordingly.
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
                    for body in self.bodies:
                        body.Froude_Krylov[:, :, idir] = body.Froude_Krylov[:, :, i360]
                        body.Diffraction[:, :, idir] = body.Diffraction[:, :, i360]

        # Sorting wave directions and creates the final FK and diffraction loads data.
        sort_dirs = np.argsort(self.wave_dir)
        self.wave_dir = self.wave_dir[sort_dirs]
        for body in self.bodies:
            body.Froude_Krylov = body.Froude_Krylov[:, :, sort_dirs]
            body.Diffraction = body.Diffraction[:, :, sort_dirs]

        self.min_wave_dir = np.min(self.wave_dir)
        self.max_wave_dir = np.max(self.wave_dir)
        self.nb_wave_dir = self.wave_dir.shape[0]

        return

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

    def eval_infinite_added_mass(self, tf = 30., dt = None,  full=True):
        """Evaluates the infinite added mass matrix coefficients using Ogilvie formula.

        Parameters
        ----------
        full : bool, optional
            If True (default), it will use the full frequency range for computations.

         It uses the Ogilvie formula to get the coefficients from the impulse response functions.
        """

        # Time.
        if dt is None:
            # Using Shannon theorem.
            dt = pi / (10 * self.max_frequency)
        time = np.arange(start=0., stop=tf + dt, step=dt)
        tf = time[-1]  # It is overwriten !!

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

    def eval_impulse_response_function_Ku(self, tf=30., dt=None, full=True):
        """Computes the impulse response functions relative to the ship advance speed

        ref : F. Rongère et al. (Journées de l'Hydrodynamique 2010 - Nantes).

        Parameters
        ----------
        tf : float, optional
            Final time (seconds). Default is 30.
        dt : float, optional
            Time step (seconds). Default is None. If None, a time step is computed according to the max
            frequency of
            hydrodynamic coefficients (the Nyquist frequency is taken)
        full : bool, optional
            If True (default), it will use the full frequency range for computations.
        """

        # Time.
        if dt is None:
            # Using Shannon theorem.
            dt = pi / (10 * self.max_frequency)
        time = np.arange(start=0., stop=tf + dt, step=dt)
        tf = time[-1]  # It is overwriten !!

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

                for body in self.bodies:
                    fk_db_temp = np.copy(body.Froude_Krylov[:, :, i])
                    fk_db_temp[(1, 3, 5), :] = -fk_db_temp[(1, 3, 5), :]
                    body.Froude_Krylov = np.concatenate((body.Froude_Krylov, fk_db_temp.reshape(6, nw, 1)), axis=2) # Axis of the wave directions.

                    diff_db_temp = np.copy(body.Diffraction[:, :, i])
                    diff_db_temp[(1, 3, 5), :] = -diff_db_temp[(1, 3, 5), :]
                    body.Diffraction = np.concatenate((body.Diffraction, diff_db_temp.reshape(6, nw, 1)), axis=2) # Axis of the wave directions.

        return