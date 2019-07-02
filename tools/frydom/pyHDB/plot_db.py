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
import matplotlib.cm as cm

Beta_report = np.array([0., 30., 60., 90., 120., 150., 180.], np.float)
Dof_notation = [r'x',r'y',r'z',r'\phi',r'\theta',r'\psi']
Dof_name = ["surge", "sway", "heave", "roll", "pitch", "yaw"]

def plot_loads(data, w, DiffOrFKOrExc, ibody, iforce, beta, show = True, save = False, filename = "Loads.png"):
    """Plots the diffraction or Froude-Krylov or excitation response function of a given modes set.

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
    """

    # Labels and title.
    xlabel = r'$\omega$'+' $(rad/s)$'
    if(DiffOrFKOrExc == 0): # Diffraction loads.
        ylabel1 = r'$|F_{Diff}(\omega, \beta)|$'
        ylabel2 = r'$Arg\left[F_{Diff}(\omega,\beta)\right] (deg)$'
        title = r'Diffraction loads in %s of body %u for the wave direction %.1f deg' % \
                (Dof_name[iforce], ibody + 1, np.degrees(beta))
    elif(DiffOrFKOrExc == 1): # Froude-Krylov loads.
        ylabel1 = r'$|F_{FK}(\omega, \beta)|$'
        ylabel2 = r'$Arg\left[F_{FK}(\omega,\beta)\right] (deg)$'
        title = r'Froude-Krylov loads in %s of body %u for the wave direction %.1f deg' % \
                (Dof_name[iforce], ibody + 1, np.degrees(beta))
    elif(DiffOrFKOrExc == 2): # Excitation loads.
        ylabel1 = r'$|F_{Exc}(\omega, \beta)|$'
        ylabel2 = r'$Arg\left[F_{Exc}(\omega,\beta)\right] (deg)$'
        title = r'Excitation loads in %s of body %u for the wave direction %.1f deg' % \
                (Dof_name[iforce], ibody + 1, np.degrees(beta))

    # Units.
    if (iforce <= 2):
        ylabel1 += r' $(N/m)$'
    else:
        ylabel1 += r' $(N)$'

    # Plots.
    if (save == False): # The size is smaller for the generation of automatic report because the title is not including.
        plt.figure(num=None, figsize=(16, 8.5))
    else:
        plt.figure(num=None, figsize=(10, 6))
    plt.subplot(2, 1, 1)
    plt.plot(w, np.absolute(data),linestyle="-", linewidth = 2)
    plt.ylabel(ylabel1, fontsize=18)
    if (save == False): # The title is not necessary for the generation of automatic report.
        plt.title(title, fontsize=20)
    plt.grid()

    plt.subplot(2, 1, 2)
    plt.plot(w, np.angle(data, deg=True),linestyle="-", linewidth = 2)
    plt.ylabel(ylabel2, fontsize=18)
    plt.xlabel(xlabel, fontsize=18)
    plt.grid()

    if (show == True):
        plt.show()
    if (save == True):
        plt.tight_layout()
        plt.savefig(filename)
        plt.close()

def plot_loads_all_wave_dir(data, w, DiffOrFKOrExc, ibody, iforce, beta, show = True, save = False, filename = "Loads.png"):
    """Plots the diffraction or Froude-Krylov or excitation response functions.

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
        Wave directions in radians.
    """

    # Labels and title.
    xlabel = r'$\omega$'+' $(rad/s)$'
    if(DiffOrFKOrExc == 0): # Diffraction loads.
        ylabel1 = r'$|F_{Diff}(\omega, \beta)|$'
        ylabel2 = r'$Arg\left[F_{Diff}(\omega,\beta)\right] (deg)$'
        title = r'Diffraction loads in %s of body %u' % \
                (Dof_name[iforce], ibody + 1)
    elif(DiffOrFKOrExc == 1): # Froude-Krylov loads.
        ylabel1 = r'$|F_{FK}(\omega, \beta)|$'
        ylabel2 = r'$Arg\left[F_{FK}(\omega,\beta)\right] (deg)$'
        title = r'Froude-Krylov loads in %s of body %u' % \
                (Dof_name[iforce], ibody + 1)
    elif(DiffOrFKOrExc == 2): # Excitation loads.
        ylabel1 = r'$|F_{Exc}(\omega, \beta)|$'
        ylabel2 = r'$Arg\left[F_{Exc}(\omega,\beta)\right] (deg)$'
        title = r'Excitation loads in %s of body %u' % \
                (Dof_name[iforce], ibody + 1)

    # Units.
    if (iforce <= 2):
        ylabel1 += r' $(N/m)$'
    else:
        ylabel1 += r' $(N)$'

    # Colors.
    colors = cm.jet(np.linspace(1, 0, beta.shape[0]))

    # Plots.
    if (save == False): # The size is smaller for the generation of automatic report because the title is not including.
        plt.figure(num=None, figsize=(16, 8.5))
    else:
        plt.figure(num=None, figsize=(10, 6))

    # Amplitude.
    plt.subplot(2, 1, 1)
    for ibeta in range(0, beta.shape[0]):
        plt.plot(w, np.absolute(data[:, ibeta]),linestyle="-", linewidth = 2, label = str(beta[ibeta])+" deg", color = colors[ibeta])
    plt.ylabel(ylabel1, fontsize=18)
    if (save == False): # The title is not necessary for the generation of automatic report.
        plt.title(title, fontsize=20)
    plt.grid()
    plt.legend()

    # Phase.
    plt.subplot(2, 1, 2)
    for ibeta in range(0, beta.shape[0]):
        plt.plot(w, np.angle(data[:, ibeta], deg=True),linestyle="-", linewidth = 2, color = colors[ibeta])
    plt.ylabel(ylabel2, fontsize=18)
    plt.xlabel(xlabel, fontsize=18)
    plt.grid()

    # Show and save.
    if (show == True):
        plt.show()
    if (save == True):
        plt.tight_layout()
        plt.savefig(filename)
        plt.close()

def plot_AB(data, w, ibody_force, iforce, ibody_motion, idof, show = True, save = False, filename = "AB.png"):
    """Plots the radiation coefficients of a given modes set.

    Parameters
    ----------
    data : Array of floats.
        Data to plot: added mass and damping coefficients.
    w : Array of floats.
        Wave frequencies.
    ibody_force : int
        Index of the body where the radiation force is applied.
    iforce : int
        Index of the local body's force mode.
    ibody_motion : int
        Index of the body having a motion.
    idof : int
        Index of the local body's radiation mode (motion).
    """

    xlabel = r'$\omega$'+' $(rad/s)$'
    ylabel1 = r'$A_{%s}(\omega)$' % (Dof_notation[iforce]+"_"+str(ibody_force+1) + Dof_notation[idof]+"_"+str(ibody_motion+1))
    ylabel2 = r'$B_{%s}(\omega)$' % (Dof_notation[iforce]+"_"+str(ibody_force+1) + Dof_notation[idof]+"_"+str(ibody_motion+1))

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

    title = r"Radiation coefficients giving %s in %s of body %u " \
            r"for a %s %s of body %u along direction %u" \
            % (force_str, Dof_name[iforce], ibody_force+1, motion_str, Dof_name[idof], ibody_motion+1)

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
        plt.close()

def plot_AB_multiple_coef(data, w, ibody_force, iforce, ibody_motion, show = True, save = False, filename = "AB.png"):
    """Plots the radiation coefficients of a given modes set.

    Parameters
    ----------
    data : Array of floats.
        Data to plot: added mass and damping coefficients.
    w : Array of floats.
        Wave frequencies.
    ibody_force : int
        Index of the body where the radiation force is applied.
    iforce : int
        Index of the local body's force mode.
    ibody_motion : int
        Index of the body having a motion.
    """

    xlabel = r'$\omega$'+' $(rad/s)$'
    ylabel1 = r'$A(\omega)$'
    ylabel2 = r'$B(\omega)$'

    if (iforce <= 2):
        force_str = 'force'
    else:
        force_str = 'moment'

    title = r"Radiation coefficients giving %s in %s of body %u  " \
            r"generated by a motion of body %u" \
            % (force_str, Dof_name[iforce], ibody_force+1, ibody_motion+1)

    plt.close()
    if(save == False): # The size is smaller for the generation of automatic report because the title is not including.
        plt.figure(num=None, figsize=(16, 8.5))
    else:
        plt.figure(num=None, figsize=(10, 6))

    # Colors.
    colors = cm.jet(np.linspace(0.9, 0, 6))

    # Added mass coefficients.
    plt.subplot(2, 1, 1)
    for idof in range(0, 6):
        unitA = r'$A_{%s}$' % (Dof_notation[iforce]+"_"+str(ibody_force+1) + Dof_notation[idof]+"_"+str(ibody_motion+1))
        if (iforce <= 2):
            if (idof <= 2): # Translation.
                unitA += r' $(kg)$'
            else: # Rotation.
                unitA += r' $(kg\,m)$'
        else:
            if (idof <= 2): # Translation.
                unitA += r' $(kg\,m)$'
            else: # Rotation.
                unitA += r' $(kg\,m^2)$'
        plt.plot(w, data[:len(w), idof], linestyle="-", linewidth = 2, label = unitA, color = colors[idof])
        plt.plot(w[-1], data[-1, idof], marker = "+", markersize = 10, mew=3, color = colors[idof])
    plt.ylabel(ylabel1, fontsize=18)
    if(save == False): # The title is not necessary for the generation of automatic report.
        plt.title(title, fontsize = 20)
    plt.grid()
    plt.legend(fontsize = 12)

    plt.subplot(2, 1, 2)
    for idof in range(0, 6):
        unitB = r'$B_{%s}$' % (Dof_notation[iforce]+"_"+str(ibody_force+1) + Dof_notation[idof]+"_"+str(ibody_motion+1))
        if (iforce <= 2):
            if (idof <= 2): # Translation.
                unitB += r' $(kg/s)$'
            else: # Rotation.
                unitB += r' $(kg\,m/s)$'
        else:
            if (idof <= 2): # Translation.
                unitB += r' $(kg\,m/s)$'
            else: # Rotation.
                unitB += r' $(kg\,m^2/s)$'
        plt.plot(w, data[0:len(w),6 + idof], linestyle="-", linewidth = 2, label = unitB, color = colors[idof])
    plt.ylabel(ylabel2, fontsize=18)
    plt.xlabel(xlabel, fontsize=18)
    plt.grid()
    plt.legend(fontsize = 12)

    if (show == True):
        plt.show()
    if(save == True):
        plt.tight_layout()
        plt.savefig(filename)
        plt.close()

def plot_irf(data, time, SpeedOrNot, ibody_force, iforce, ibody_motion, idof, show = True, save = False, filename = "IRF.png"):
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
        Index of the body where the radiation force is applied.
    iforce : int
        Index of the local body's force mode.
    ibody_motion : int
        Index of the body having a motion.
    idof : int
        Index of the local body's raditation mode (motion).
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
        ylabel = r'$K_{%s}$' % (Dof_notation[iforce]+"_"+str(ibody_force+1) + Dof_notation[idof]+"_"+str(ibody_motion+1))
    else: # With forward speed.
        ylabel = r'$Ku_{%s}$' % (Dof_notation[iforce]+"_"+str(ibody_force+1) + Dof_notation[idof]+"_"+str(ibody_motion+1))

    # Plots.
    if (save == False):  # The size is smaller for the generation of automatic report because the title is not including.
        plt.figure(num=None, figsize=(16, 8.5))
    else:
        plt.figure(num=None, figsize=(10, 6))
    plt.plot(time, data)
    plt.xlabel(r'$t$'+' $(s)$', fontsize=18)
    plt.ylabel(ylabel, fontsize=18)  # TODO: mettre une unite
    if (save == False): # The title is not necessary for the generation of automatic report.
        if(SpeedOrNot == 0): # Without forward speed.
            plt.title('Impulse response function of %s in %s of body %u for a %s %s of body %u' %
                  (force_str, Dof_name[iforce], ibody_force + 1, Dof_name[idof], motion_str, ibody_motion + 1), fontsize = 20)
        else: # With forward speed.
            plt.title('Impulse response function with forward speed of %s in %s of body %u for a %s %s of body %u' %
                      (force_str, Dof_name[iforce], ibody_force + 1, iforce + 1, Dof_name[idof], motion_str, ibody_motion + 1), fontsize=20)
    plt.grid()

    if (show == True):
        plt.show()
    if (save == True):
        plt.tight_layout()
        plt.savefig(filename)
        plt.close()

def plot_irf_multiple_coef(data, time, SpeedOrNot, ibody_force, iforce, ibody_motion, show = True, save = False, filename = "IRF.png"):
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
        Index of the body where the radiation force is applied.
    iforce : int
        Index of the local body's force mode.
    ibody_motion : int
        Index of the body having a motion.
    """

    # Labels.
    if (iforce <= 2):
        force_str = 'force'
    else:
        force_str = 'moment'

    if (SpeedOrNot == 0): # Without forward speed.
        ylabel = r'$K(t)$'
    else: # With forward speed.
        ylabel = r'$Ku(t)$'

    # Colors.
    colors = cm.jet(np.linspace(0.9, 0, 6))

    # Plots.
    if (save == False): # The size is smaller for the generation of automatic report because the title is not including.
        plt.figure(num=None, figsize=(16, 8.5))
    else:
        plt.figure(num=None, figsize=(10, 6))
    for idof in range(0, 6):
        if (SpeedOrNot == 0): # Without forward speed.
            unit = r'$K_{%s}(t)$' % (Dof_notation[iforce]+"_"+str(ibody_force+1) + Dof_notation[idof]+"_"+str(ibody_motion+1))
        else: # With forward speed.
            unit = r'$Ku_{%s}(t)$' % (Dof_notation[iforce]+"_"+str(ibody_force+1) + Dof_notation[idof]+"_"+str(ibody_motion+1))
        plt.plot(time, data[:, idof], linestyle="-", linewidth=2, label = unit, color = colors[idof])
    plt.xlabel(r'$t$'+' $(s)$', fontsize=18)
    plt.ylabel(ylabel, fontsize=18) # TODO: mettre une unite
    if (save == False): # The title is not necessary for the generation of automatic report.
        if(SpeedOrNot == 0): # Without forward speed.
            plt.title('Impulse response function of %s in %s of body %u for a motion of body %u' %
                  (force_str, Dof_name[iforce], ibody_force + 1, ibody_motion + 1), fontsize = 20)
        else: # With forward speed.
            plt.title('Impulse response function with forward speed of %s in %s of body %u for a motion of body %u' %
                      (force_str, Dof_name[iforce], ibody_force + 1, ibody_motion + 1), fontsize=20)
    plt.grid()
    plt.legend(fontsize = 14)

    if (show == True):
        plt.show()
    if (save == True):
        plt.tight_layout()
        plt.savefig(filename)
        plt.close()

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
        Index of the body where the radiation force is applied.
    iforce : int
        Index of the local body's force mode.
    ibody_motion : int
        Index of the body having a motion.
    idof : int
        Index of the local body's raditation mode (motion).
    kwargs: optional
        Arguments that are to be used by pyplot.
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

    plt.figure()
    plt.plot(time, data, label="Without filetering")
    plt.plot(time, data * coeff, label="With filering")
    plt.xlabel(r'$t$'+' $(s)$', fontsize=18)
    plt.ylabel(ylabel, fontsize=18)  # TODO: mettre une unite
    if (SpeedOrNot == 0): # Without forward speed.
        plt.title('Impulse response function of %s in %s of body %u along direction %u for a %s %s of body %u' %
                  (force_str, Dof_name[iforce], ibody_force + 1, Dof_name[idof], motion_str, ibody_motion + 1), fontsize=20)
    else: # With forward speed.
        plt.title('Impulse response function with forward speed of %s in %s of body %u along direction %u for a %s %s of body %u' %
                  (force_str, Dof_name[iforce], ibody_force + 1, Dof_name[idof], motion_str, ibody_motion + 1), fontsize=20)
    plt.legend()
    plt.grid()
    plt.show()