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
        self.nb_dir_kochin = 0
        self.min_dir_kochin = 0.
        self.max_dir_kochin = 0.
        self.wave_dir_Kochin = np.array([])

        # Bodies.
        self.nb_bodies = 0
        self.bodies = []

        # Version.
        self.version = 2.0

        # Froude-Krylov loads.
        self._has_froude_krylov = False

    def set_wave_frequencies(self):
        """Frequency array of BEM computations in rad/s

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