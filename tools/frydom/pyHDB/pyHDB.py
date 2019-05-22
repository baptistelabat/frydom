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

from wave_dispersion_relation_v2 import solve_wave_dispersion_relation

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