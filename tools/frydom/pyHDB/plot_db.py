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

"""Module to plot the hydrodynamic database."""

import numpy as np
import matplotlib.pyplot as plt

def plot_loads(data, w, DiffOrFKOrExc, ibody, iforce, beta, **kwargs):
    """Plots the diffraction or Froude-Krylov response function of a given modes set

    Parameters
    ----------
    data : Array of floats.
        Data to plot: diffraction or Froude-Krylov loads.
    w : Array of floats.
        Wave frequencies.
    DiffOrFKOrExc : int.
        0 for diffraction loads, 1 for Froude-Krylov loads, 2 for excitation loads.
    ibody : int.
        The index of the body.
    iforce : int.
        The index of the body's force mode.
    beta : float.
        Wave direction in radians.
    kwargs: optional.
        Arguments that are to be used by pyplot
    """

    # Labels and title.
    xlabel = r'$\omega$'+' $(rad/s)$'
    if(DiffOrFKOrExc == 0): # Diffraction loads.
        ylabel1 = r'$|F_{Diff}(\omega, \beta)|$'
        ylabel2 = r'$Arg\left[F_{Diff}(\omega,\beta)\right] (deg)$'
        title = r'Diffraction loads on body %u along the direction %u for the wave direction %.1f deg' % \
                (ibody+1, iforce+1, np.degrees(beta))
    elif(DiffOrFKOrExc == 1): # Froude-Krylov loads.
        ylabel1 = r'$|F_{FK}(\omega, \beta)|$'
        ylabel2 = r'$Arg\left[F_{FK}(\omega,\beta)\right] (deg)$'
        title = r'Froude-Krylov loads on body %u along the direction %u for the wave direction %.1f deg' % \
                (ibody+1, iforce+1, np.degrees(beta))
    elif(DiffOrFKOrExc == 2): # Excitation loads.
        ylabel1 = r'$|F_{Exc}(\omega, \beta)|$'
        ylabel2 = r'$Arg\left[F_{Exc}(\omega,\beta)\right] (deg)$'
        title = r'Excitation loads on body %u along the direction %u for the wave direction %.1f deg' % \
                (ibody + 1, iforce + 1, np.degrees(beta))

    # Units.
    if (iforce <= 2):
        ylabel1 += r' $(N/m)$'
    else:
        ylabel1 += r' $(N)$'

    # Plots.
    plt.subplot(2, 1, 1)
    plt.plot(w, np.absolute(data),linestyle="-", linewidth = 2)
    plt.ylabel(ylabel1, fontsize=18)
    plt.title(title, fontsize = 20)
    plt.grid()

    plt.subplot(2, 1, 2)
    plt.plot(w, np.angle(data, deg=True),linestyle="-", linewidth = 2)
    plt.ylabel(ylabel2, fontsize=18)
    plt.xlabel(xlabel, fontsize=18)
    plt.grid()

    plt.show()

def plot_AB(data, w, ibody_force, iforce, ibody_motion, idof, show = True, save = False, filename = "Figure.png"):
    """Plots the radiation coefficients of a given modes set.

    Parameters
    ----------
    data : Array of floats.
        Data to plot: added mass and damping coefficients.
    w : Array of floats.
        Wave frequencies.
    ibody_force : int
        Index of the body where the radiation force is applied
    iforce : int
        Index of the local body's force mode
    ibody_motion : int
        Index of the body having a motion
    idof : int
        Index of the local body's radiation mode (motion)
    """

    xlabel = r'$\omega$'+' $(rad/s)$'
    ylabel1 = r'$A_{%s}(\omega)$' % (str(iforce+1) + str(idof+1))
    ylabel2 = r'$B_{%s}(\omega)$' % (str(iforce+1) + str(idof+1))

    if (iforce <= 2):
        force_str = 'force'
        if (idof <= 2): # Translation.
            ylabel1 += r' $(kg)$'
            ylabel2 += r' $(kg/s)$'
            motion_str = 'translation'
        else: # Rotation.
            ylabel1 += r' $(kg\,m)$'
            ylabel2 += r' $(kg\,m/s)$'
            motion_str = 'rotation'
    else:
        force_str = 'moment'
        if (idof <= 2): # Translation.
            ylabel1 += r' $(kg\,m)$'
            ylabel2 += r' $(kg\,m/s)$'
            motion_str = 'translation'
        else: # Rotation.
            ylabel1 += r' $(kg\,m^2)$'
            ylabel2 += r' $(kg\,m^2/s)$'
            motion_str = 'rotation'

    title = r"Radiation coefficients giving %s on body %u along direction %u " \
            r"for a %s of body %u along direction %u" \
            % (force_str, ibody_force+1, iforce+1, motion_str, ibody_motion+1, idof+1)

    plt.close()
    if(save == False): # The size is smaller for the generation of automatic report because the title is not including.
        plt.figure(num=None, figsize=(16, 8.5))
    else:
        plt.figure(num=None, figsize=(10, 6))
    plt.subplot(2, 1, 1)
    plt.plot(w, data[:len(w),0],linestyle="-", linewidth = 2)
    plt.plot(w[-1], data[-1, 0],marker = "+", color= "red", markersize = 10)
    plt.ylabel(ylabel1, fontsize=18)
    if(save == False): # The title is not necessary for the generation of automatic report.
        plt.title(title, fontsize = 20)
    plt.grid()

    plt.subplot(2, 1, 2)
    plt.plot(w, data[0:len(w),1],linestyle="-", linewidth = 2)
    plt.ylabel(ylabel2, fontsize=18)
    plt.xlabel(xlabel, fontsize=18)
    plt.grid()

    if (show == True):
        plt.show()
    if(save == True):
        plt.tight_layout()
        plt.savefig(filename)

def plot_irf(data, time, SpeedOrNot, ibody_force, iforce, ibody_motion, idof, **kwargs):
    """Plots the impulse response function of a given modes set.

    Parameters
    ----------
    data : Array of floats.
        Data to plot: impulse response functions.
    time : Array of floats.
        Time.
    SpeedOrNot : int
        IRF with forward speed (1) or not (0).
    ibody_force : int
        Index of the body where the radiation force is applied
    iforce : int
        Index of the local body's force mode
    ibody_motion : int
        Index of the body having a motion
    idof : int
        Index of the local body's raditation mode (motion)
    kwargs: optional
        Arguments that are to be used by pyplot
    """

    # Labels.
    if (iforce <= 2):
        force_str = 'force'
        if (idof <= 2): # Translation.
            motion_str = 'translation'
        else: # Rotation.
            motion_str = 'rotation'
    else:
        force_str = 'moment'
        if (idof <= 2): # Translation.
            motion_str = 'translation'
        else: # Rotation.
            motion_str = 'rotation'

    if (SpeedOrNot == 0): # Without forward speed.
        ylabel = r'$K_{%s}(t)$' % (str(iforce+1) + str(idof+1))
    else: # With forward speed.
        ylabel = r'$Ku_{%s}(t)$' % (str(iforce + 1) + str(idof + 1))

    plt.plot(time, data, **kwargs)
    plt.xlabel(r'$t$'+' $(s)$', fontsize=18)
    plt.ylabel(ylabel, fontsize=18)  # TODO: mettre une unite
    if(SpeedOrNot == 0): # Without forward speed.
        plt.title('Impulse response function of %s on body %u along direction %u for a %s of body %u along direction %u' %
              (force_str, ibody_force+1, iforce+1, motion_str, ibody_motion+1, idof+1), fontsize = 20)
    else: # With forward speed.
        plt.title('Impulse response function with forward speed of %s on body %u along direction %u for a %s of body %u along direction %u' %
                  (force_str, ibody_force + 1, iforce + 1, motion_str, ibody_motion + 1, idof + 1), fontsize=20)
    plt.grid()
    plt.show()

def plot_filering(data, time, SpeedOrNot, coeff, ibody_force, iforce, ibody_motion, idof, **kwargs):
    """This function plots the filtered impulse response functions.

    Parameters
    ----------
    data : Array of floats.
        Data to plot: impulse response functions with and without filetering.
    time : Array of floats.
        Time.
    SpeedOrNot : int
        IRF with forward speed (1) or not (0).
    coeff : Array of floats.
        Filerting.
    SpeedOrNot : int
        IRF with forward speed (1) or not (0).
    ibody_force : int
        Index of the body where the radiation force is applied
    iforce : int
        Index of the local body's force mode
    ibody_motion : int
        Index of the body having a motion
    idof : int
        Index of the local body's raditation mode (motion)
    kwargs: optional
        Arguments that are to be used by pyplot
    """

    # Labels.
    if (iforce <= 2):
        force_str = 'force'
        if (idof <= 2):  # Translation.
            motion_str = 'translation'
        else:  # Rotation.
            motion_str = 'rotation'
    else:
        force_str = 'moment'
        if (idof <= 2):  # Translation.
            motion_str = 'translation'
        else:  # Rotation.
            motion_str = 'rotation'

    if (SpeedOrNot == 0): # Without forward speed.
        ylabel = r'$K_{%s}(t)$' % (str(iforce+1) + str(idof+1))
    else: # With forward speed.
        ylabel = r'$Ku_{%s}(t)$' % (str(iforce + 1) + str(idof + 1))

    plt.figure()
    plt.plot(time, data, label="Without filetering")
    plt.plot(time, data * coeff, label="With filering")
    plt.xlabel(r'$t$'+' $(s)$', fontsize=18)
    plt.ylabel(ylabel, fontsize=18)  # TODO: mettre une unite
    if (SpeedOrNot == 0):  # Without forward speed.
        plt.title('Impulse response function of %s on body %u along direction %u for a %s of body %u along direction %u' %
                  (force_str, ibody_force + 1, iforce + 1, motion_str, ibody_motion + 1, idof + 1), fontsize=20)
    else:  # With forward speed.
        plt.title('Impulse response function with forward speed of %s on body %u along direction %u for a %s of body %u along direction %u' %
                  (force_str, ibody_force + 1, iforce + 1, motion_str, ibody_motion + 1, idof + 1), fontsize=20)
    plt.legend()
    plt.grid()
    plt.show()