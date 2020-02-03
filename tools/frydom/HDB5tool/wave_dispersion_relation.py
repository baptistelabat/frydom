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

from math import log, isinf, fabs, sqrt, tanh
import numpy as np


def solve_wave_dispersion_relation(omega, depth, grav):
    """Solves the dispersion relation of linear waves propagation

    Parameters
    ----------
    omega : float, array_like
        angular frequency vector (rad/s)
    depth : float
        water depth (meters)
    grav : float
        gravity acceleration (m/s**2)

    Returns
    -------
    float, np.ndarray
        The wave number vector
    """

    if np.isscalar(omega):
        omega = np.array([omega], dtype=np.float)
    else:
        omega = np.asarray(omega, dtype=np.float)
    omega2_g = omega * omega / grav

    if isinf(depth):
        return omega2_g

    omega2h_g = omega2_g * depth

    prec = 1e-6
    b_min = 1e-3
    b_max = -log(prec * 0.5) * 0.5
    a = 1. / 3.
    b = 2. / 15.
    c = 17. / 315.
    d = 62. / 2835.
    e = 1382. / 155925.

    def xtanhx(x):
        """To solve the equation y = x tanh(x) robustly"""
        if x == 0.:
            return 0.

        if x > b_max:  # Tiny wave length, depth is like infinite
            return x

        elif x < b_min:  # Large wave length, we need a taylor serie
            dt = ds = 0.
            while True:
                y = ds + x
                y2 = y * y
                ds = y2 * (a - b * y + c * y2 - d * y2 * y + e * y2 * y2)
                dprec = fabs((ds - dt) / (2. * y))
                if (dprec - prec) < 0.:
                    return sqrt(y)
                dt = ds

        else:  # Normal wave length
            # Using bisection algorithm to find the solution
            y = x
            while True:
                dx = x / tanh(y)
                if fabs((dx - y) / x) < prec:
                    return y
                y = dx

    return np.asarray(list(map(xtanhx, omega2h_g)), dtype=np.float) / depth
