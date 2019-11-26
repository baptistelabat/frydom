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
import copy

import meshmagick.MMviewer

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

        # Amplitude.
        if(iforce <= 2):
            ylabel1 = r'$|F_{Diff}^{%s}(\omega, \beta)|$' % Dof_notation[iforce]
        else:
            ylabel1 = r'$|M_{Diff}^{%s}(\omega, \beta)|$' % Dof_notation[iforce]

        # Phase.
        if (iforce <= 2):
            ylabel2 = r'$Arg\left[F_{Diff}^{%s}(\omega,\beta)\right] (deg)$'% Dof_notation[iforce]
        else:
            ylabel2 = r'$Arg\left[F_{Diff}^{%s}(\omega,\beta)\right] (deg)$'% Dof_notation[iforce]

        # Title.
        title = r'Diffraction loads in %s of body %u for a wave of direction %.1f deg' % \
                (Dof_name[iforce], ibody + 1, np.degrees(beta))
    elif(DiffOrFKOrExc == 1): # Froude-Krylov loads.

        # Amplitude.
        if (iforce <= 2):
            ylabel1 = r'$|F_{FK}^{%s}(\omega, \beta)|$' % Dof_notation[iforce]
        else:
            ylabel1 = r'$|M_{FK}^{%s}(\omega, \beta)|$' % Dof_notation[iforce]

        # Phase.
        if(iforce <= 2):
            ylabel2 = r'$Arg\left[F_{FK}^{%s}(\omega,\beta)\right] (deg)$' % Dof_notation[iforce]
        else:
            ylabel2 = r'$Arg\left[F_{FK}^{%s}(\omega,\beta)\right] (deg)$' % Dof_notation[iforce]

        # Title.
        title = r'Froude-Krylov loads in %s of body %u for a wave of direction %.1f deg' % \
                (Dof_name[iforce], ibody + 1, np.degrees(beta))
    elif(DiffOrFKOrExc == 2): # Excitation loads.

        # Amplitude.
        if (iforce <= 2):
            ylabel1 = r'$|F_{Exc}^{%s}(\omega, \beta)|$' % Dof_notation[iforce]
        else:
            ylabel1 = r'$|M_{Exc}^{%s}(\omega, \beta)|$' % Dof_notation[iforce]

        # Phase.
        if(iforce <= 2):
            ylabel2 = r'$Arg\left[F_{Exc}^{%s}(\omega,\beta)\right] (deg)$' % Dof_notation[iforce]
        else:
            ylabel2 = r'$Arg\left[F_{Exc}^{%s}(\omega,\beta)\right] (deg)$' % Dof_notation[iforce]

        # Title.
        title = r'Excitation loads in %s of body %u for a wave of direction %.1f deg' % \
                (Dof_name[iforce], ibody + 1, np.degrees(beta))

    # Units.
    if (iforce <= 2):
        ylabel1 += r' $(N/m)$'
    else:
        ylabel1 += r' $(N)$'

    # Plots.
    if (save == False):
        plt.figure(num=None, figsize=(16, 8.5))
    else:
        plt.figure(num=None, figsize=(10, 6))
    plt.subplot(2, 1, 1)
    plt.plot(w, np.absolute(data),linestyle="-", linewidth = 2)
    plt.ylabel(ylabel1, fontsize=18)
    if (save == False):
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

    # Label.
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

    title = r"Radiation coefficients giving the %s in %s of body %u " \
            r"for a %s in %s of body %u" \
            % (force_str, Dof_name[iforce], ibody_force + 1, motion_str, Dof_name[idof], ibody_motion + 1)

    plt.close()
    if(save == False):
        plt.figure(num=None, figsize=(16, 8.5))
    else:
        plt.figure(num=None, figsize=(10, 6))

    # Added mass.
    plt.subplot(2, 1, 1)
    plt.plot(w, data[:len(w),0],linestyle="-", linewidth = 2)
    plt.plot(w[-1], data[-1, 0],marker = "+", color= "red", markersize = 10)
    plt.ylabel(ylabel1, fontsize=18)
    if(save == False):
        plt.title(title, fontsize = 20)
    plt.grid()

    # Damping.
    plt.subplot(2, 1, 2)
    plt.plot(w, data[0:len(w),1],linestyle="-", linewidth = 2)
    plt.ylabel(ylabel2, fontsize=18)
    plt.xlabel(xlabel, fontsize=18)
    plt.grid()

    # Show and save.
    if (show == True):
        plt.show()
    if(save == True):
        plt.tight_layout()
        plt.savefig(filename)
    plt.close()

def plot_AB_array(data, w, ibody_force, ibody_motion, pyHDB):
    """Plots ALL the radiation coefficients of a body.

    Parameters
    ----------
    data : Array of floats.
        Data to plot: Combinaison of both added mass and damping coefficients.
    w : Array of floats.
        Wave frequencies.
    ibody_force : int
        Index of the body where the radiation force is applied.
    ibody_motion : int
        Index of the body having a motion.
    """

    # Title.
    title = r"Combinaison of radiation coefficients of body %u generated by a motion of body %u" % (ibody_force + 1, ibody_motion + 1) \
            + "\n " + \
            r"$H_{%s}(j\omega) = |B_{%s}(\omega) + j\omega[A_{%s}(\omega) - A_{%s}^{\infty}]|$" \
            % (str(ibody_force + 1) + str(ibody_motion + 1), str(ibody_force + 1) + str(ibody_motion + 1),
               str(ibody_force + 1) + str(ibody_motion + 1), str(ibody_force + 1) + str(ibody_motion + 1))

    # Definition of the figure.
    plt.close()
    fig, axes = plt.subplots(6, 6, figsize=(16, 8.5))

    # Plot.
    for iforce in range(0, 6):
        for idof in range(0, 6):
            labelA = r'$H_{%s} (\omega)$' % (Dof_notation[iforce] + "_" + str(ibody_force + 1) + Dof_notation[idof] + "_" + str(ibody_motion + 1))
            axes[iforce, idof].plot(w, data[iforce, idof, :], linestyle="-", linewidth=2)
            axes[iforce, idof].grid()
            axes[iforce, idof].set_title(labelA)

            # Automatic pre-filling of the radiation mask (can be change by clicking on the plots).
            if (pyHDB.bodies[ibody_force].Radiation_mask[iforce, 6 * ibody_motion + idof] == True
                    and (pyHDB.bodies[ibody_force].Force_mask[iforce] == 0 or pyHDB.bodies[ibody_force].Motion_mask[idof] == 0)):
                pyHDB.bodies[ibody_force].Radiation_mask[iforce, 6 * ibody_motion + idof] = False
                axes[iforce, idof].set_facecolor("grey")

            # If a radiation mask has already been defined.
            if (pyHDB.bodies[ibody_force].Radiation_mask[iforce, 6 * ibody_motion + idof] == False):
                axes[iforce, idof].set_facecolor("grey")

    # What to do if a mouse click is performed.
    def onclick(event):

        # Change the boolean of Radiation_mask.
        iplot = 0
        for iforce in range(0, 6):
            for idof in range(0, 6):
                if event.inaxes == axes[iforce, idof]:

                    # Inverse the radiation mask and update the background color of the plot accordingly.
                    if(pyHDB.bodies[ibody_force].Radiation_mask[iforce, 6 * ibody_motion + idof] == True):
                        pyHDB.bodies[ibody_force].Radiation_mask[iforce, 6 * ibody_motion + idof] = False
                        event.canvas.figure.get_axes()[iplot].set_facecolor('grey')
                    else:
                        pyHDB.bodies[ibody_force].Radiation_mask[iforce, 6 * ibody_motion + idof] = True
                        event.canvas.figure.get_axes()[iplot].set_facecolor('white')

                    # Application of the modification.
                    event.canvas.draw()

                iplot = iplot + 1

    # Even of a mouse click.
    cid = fig.canvas.mpl_connect('button_press_event', onclick)

    # Title.
    plt.suptitle(title)

    plt.tight_layout(rect=[0, 0.03, 1, 0.95])

    # Show the plot.
    plt.show()

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
    if (SpeedOrNot == 0): # Without forward speed.
        ylabel = r'$K_{%s}$' % (Dof_notation[iforce]+"_"+str(ibody_force+1) + Dof_notation[idof]+"_"+str(ibody_motion+1))
    else: # With forward speed.
        ylabel = r'$Ku_{%s}$' % (Dof_notation[iforce]+"_"+str(ibody_force+1) + Dof_notation[idof]+"_"+str(ibody_motion+1))

    if (iforce <= 2):
        force_str = 'force'
        if (idof <= 2): # Translation.
            ylabel += r' $(kg/s^2)$'
            motion_str = 'translation'
        else: # Rotation.
            ylabel += r' $(kg\,m/s^2)$'
            motion_str = 'rotation'
    else:
        force_str = 'moment'
        if (idof <= 2): # Translation.
            ylabel += r' $(kg\,m/s^2)$'
            motion_str = 'translation'
        else: # Rotation.
            ylabel += r' $(kg\,m^2/s^2)$'
            motion_str = 'rotation'

    # Plots.
    if (save == False):
        plt.figure(num=None, figsize=(16, 8.5))
    else:
        plt.figure(num=None, figsize=(10, 6))
    plt.plot(time, data)
    plt.xlabel(r'$t$'+' $(s)$', fontsize=18)
    plt.ylabel(ylabel, fontsize=18)  # TODO: mettre une unite
    if (save == False):
        if(SpeedOrNot == 0): # Without forward speed.
            plt.title('Impulse response function of the radiation %s in %s of body %u for a %s in %s of body %u' %
                  (force_str, Dof_name[iforce], ibody_force + 1, motion_str, Dof_name[idof], ibody_motion + 1), fontsize = 20)
        else: # With forward speed.
            plt.title('Impulse response function with forward speed of the radiation %s in %s of body %u for a %s in %s of body %u' %
                      (force_str, Dof_name[iforce], ibody_force + 1, motion_str, Dof_name[idof], ibody_motion + 1), fontsize=20)
    plt.grid()

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
        plt.title('Impulse response function of the %s in %s of body %u for a %s in %s of body %u' %
                  (force_str, Dof_name[iforce], ibody_force + 1, motion_str, Dof_name[idof], ibody_motion + 1), fontsize=20)
    else: # With forward speed.
        plt.title('Impulse response function with forward speed of the %s in %s of body %u for a %s in %s of body %u' %
                  (force_str, Dof_name[iforce], ibody_force + 1, motion_str, Dof_name[idof], ibody_motion + 1), fontsize=20)
    plt.legend()
    plt.grid()
    plt.show()

def Meshmagick_viewer(mesh):
    """This function plots a mesh."""

    mesh_vizu = copy.deepcopy(mesh)
    vtk_polydata = mesh_vizu._vtk_polydata()
    mesh_vizu.viewer = meshmagick.MMviewer.MMViewer()
    mesh_vizu.viewer.add_polydata(vtk_polydata)
    mesh_vizu.viewer.renderer.ResetCamera()
    mesh_vizu.viewer.render_window.Render()
    mesh_vizu.viewer.render_window_interactor.Start()
    mesh_vizu.viewer.finalize()