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
    ArgParse module of Nemoh2HDB.
"""

import os
import argparse

try:
    import argcomplete

    acok = True
except:
    acok = False

parser = argparse.ArgumentParser(
    description="""  --  HDB5tool  --  
    A Python module and a command line utility to add write HDB5 file from Nemoh output files.\n\n  Example of use:\n\n  hdb5tool -ed CE""",
    formatter_class=argparse.RawDescriptionHelpFormatter)

# Edition of FRyDoM.
parser.add_argument('--edition','-ed', action="store",help="""
            Edition of FRyDoM: CE or ce for FRyDoM-CE, EE or ee for FRyDoM-EE.""")

parser.add_argument('-ee','-EE', action="store_true",help="""
            Use of FRyDoM EE.""")

parser.add_argument('-ce','-CE', action="store_true",help="""
            Use of FRyDoM CE.""")

# Path to Nemoh.cal.
parser.add_argument('--path_to_nemoh_cal','-cal', action="store",help="""
            Path to the folder including the file Nemoh.cal.""")

# Discretization - Wave directions.
parser.add_argument('--discretization_waves','--discretization_wave','-dis_waves', '-dis_wave', '-dw', action="store",help="""
            Integer for the new discretization of the wave directions.""")

# Discretization - Wave frequencies.
parser.add_argument('--discretization_frequencies','--discretization_freq','--discretization_frequency','-dis_freq','-dis_frequency','-dis_frequencies','-df', action="store",help="""
            Integer for the new discretization of the wave frequencies.""")

# Discretization - Wave frequencies.
parser.add_argument('--final_time_irf','--final_time', '-ft', action="store",help="""
            Final time for the computation of the impulse response functions.""")

# Body - Activate hydrostatics (useless).
parser.add_argument('--activate_hydrostatics','--activate_hydrostatic', '-active_hs', '-activate_hs', action="append",help="""
            Activate hydrostatics for the body of index given in argument.""")

# Body - Hydrostatic matrix.
parser.add_argument('--hydrostatics','--hydrostatic', '-hs', nargs=7, metavar=('id', 'k33', 'k44', 'k55', 'k34', 'k35', 'k45'), action="append",help="""
            Hydrostatic coefficients (K33, K44, K55, K34, K35, K45) for the body of index given in first argument.""")

# Body - Activate inertia (useless).
parser.add_argument('--activate_inertia', '-active_i', action="append",help="""
            Activate inertia for the body of index given in argument.""")

# Body - Inertia matrix.
parser.add_argument('--inertia', '-i', nargs=8, metavar=('id', 'mass', 'i44', 'i55', 'i66', 'i45', 'i46', 'i56'), action="append",help="""
            Inertia coefficients (Mass, I44, I55, I66, I45, I46, I56) for the body of index given in first argument.""")

# Body - Inertia matrix.
parser.add_argument('--inertia_only', '-io', nargs=7, metavar=('id', 'i44', 'i55', 'i66', 'i45', 'i46', 'i56'), action="append",help="""
            Inertia coefficients (I44, I55, I66, I45, I46, I56) for the body of index given in first argument.""")

# Body - Mass.
parser.add_argument('--mass', '--Mass', '-m', nargs=2, metavar=('id', 'mass'), action="append",help="""
            Mass of the body of index given in argument.""")

# Response Amplitude Operators.
parser.add_argument('--RAO', '-RAO', '--rao', '-rao', action="store_true",help="""
            Compute the response amplitude operators (FRyDoM EE only).""")

# Mean wave drift loads.
parser.add_argument('--drift','--Drift', '-d', action="store_true",help="""
            Compute the mean wave drift loads from the Kochin function (FRyDoM EE only).""")

# No symmetrization of the HDB.
parser.add_argument('--no_sym_hdb','-ns', action="store_true",help="""
            No symmetrization of the HDB.""")

# Writing the hdb5 output file.
parser.add_argument('--write', '--export','-w', action="store",help="""
            Writing the hdb5 output file with the given name.""")

# Plot added mass and damping coefficients.
parser.add_argument('--plot_radiation', '--plots_radiation', '--plot_AB','-pab', '-prad', nargs = 4, metavar=('ibody_force, iforce, ibody_motion, idof'), action="append",help="""
            Plot the added mass and damping coefficients of ibody along iforce for a motion of ibody_motion along idof.""")

# Plot diffraction loads.
parser.add_argument('--plot_diffraction', '--plots_diffraction', '--plot_diff','-pd', '-pdiff', nargs = 3, metavar=('ibody, iforce, iwave'), action="append",help="""
            Plot the diffraction loads of ibody along iforce for iwave.""")

# Plot Froude-Krylov loads.
parser.add_argument('--plot_froude_krylov', '--plots_froude_krylov', '--plot_fk','-pfk', nargs = 3, metavar=('ibody, iforce, iwave'), action="append",help="""
            Plot the Froude-Krylov loads of ibody along iforce for iwave.""")

# Plot excitation loads.
parser.add_argument('--plot_excitation', '--plots_excitation', '--plot_exc','-pe', '-pexc', nargs = 3, metavar=('ibody, iforce, iwave'), action="append",help="""
            Plot the excitation loads of ibody along iforce for iwave.""")

# Plot IRF.
parser.add_argument('--plot_irf', '--plots_irf','-pirf', nargs = 4, metavar=('ibody_force, iforce, ibody_motion, idof'), action="append",help="""
            Plot the impulse response functions of ibody along iforce for a motion of ibody_motion along idof.""")

# Plot IRF speed.
parser.add_argument('--plot_irf_speed', '--plots_irf_speed', '--plot_irf_ku','-pirfs','-pirfku', '-pirf_speed', nargs = 4, metavar=('ibody_force, iforce, ibody_motion, idof'), action="append",help="""
            Plot the impulse response functions with speed velocity of ibody along iforce for a motion of ibody_motion along idof.""")

def main():

    ####################################################################################################################
    #                                               Argparse.
    ####################################################################################################################

    if acok:
        argcomplete.autocomplete(parser)

    args, unknown = parser.parse_known_args()

    # Edition.
    if (args.edition is not None and args.ce is False and args.ee is False):
        Edition = args.edition
        if(Edition == 'CE' or Edition == 'GPL' or Edition == 'Community' or Edition == 'community' or Edition == 'ce' or Edition == 'gpl'):
            Edition = 'CE'
        elif(Edition == 'EE' or Edition == 'Entreprise' or Edition == 'entreprise' or Edition == 'ee'):
            Edition = 'EE'
        else:
            print("The edition given in input is not known, please choose between CE or EE.")
    elif(args.edition is None and args.ce is True and args.ee is False):
        Edition = 'CE'
    elif(args.edition is None and args.ce is False and args.ee is True):
        Edition = 'EE'
    else:
        print("The edition of FRyDoM is required. Please use --edition or -ed.")
        exit()

    if(Edition == 'CE'):
        from frydom.pyHDB.HDB5_v2 import *
    elif(Edition == 'EE'):
        try:
            from frydom_ee.FrHDB_ee_v2 import *
        except ImportError:
            print("Please check you have access to FRyDoM-EE, and, if yes, if the path to the deposit is in your PATH variable.")
            exit()

    # BEM reader.
    if(args.path_to_nemoh_cal is not None):
        database = HDB5()
        database.nemoh_reader(args.path_to_nemoh_cal)
    else:
        print("The path to folder including the file Nemoh.cal is required.")

    # Discretization - Wave directions.
    if(args.discretization_waves is not None):
        database.discretization.nb_wave_directions = int(args.discretization_waves)

    # Discretization - Wave frequencies.
    if (args.discretization_frequencies is not None):
        database.discretization.nb_frequencies = int(args.discretization_frequencies)

    # Discretization - Final time for IRF.
    if (args.activate_hydrostatics is not None):
        database.discretization._final_time = float(args.final_time_irf)

    # Initialize.
    database._initialize()

    # Body - Active hydrostatics (useless).
    if (args.activate_hydrostatics is not None):
        nb_activation_hydrostatics = len(args.activate_hydrostatics)
        for id in range(0,nb_activation_hydrostatics):
            database.body[int(args.activate_hydrostatics[id])-1].activate_hydrostatic()

    # Body - Hydrostatic matrix.
    if (args.hydrostatics is not None):
        nb_hydrostatics = len(args.hydrostatics)
        for j in range(0, nb_hydrostatics):
            database.body[int(args.hydrostatics[j][0]) - 1].activate_hydrostatic()
            database.body[int(args.hydrostatics[j][0]) - 1].hydrostatic.k33 = args.hydrostatics[j][1]
            database.body[int(args.hydrostatics[j][0]) - 1].hydrostatic.k44 = args.hydrostatics[j][2]
            database.body[int(args.hydrostatics[j][0]) - 1].hydrostatic.k55 = args.hydrostatics[j][3]
            database.body[int(args.hydrostatics[j][0]) - 1].hydrostatic.k34 = args.hydrostatics[j][4]
            database.body[int(args.hydrostatics[j][0]) - 1].hydrostatic.k35 = args.hydrostatics[j][5]
            database.body[int(args.hydrostatics[j][0]) - 1].hydrostatic.k45 = args.hydrostatics[j][6]

    # Body - Active inertia (useless).
    if (args.activate_inertia is not None):
        nb_activation_inertia = len(args.activate_inertia)
        for id in range(0, nb_activation_inertia):
            database.body[int(args.activate_inertia[id]) - 1].activate_inertia()

    # Body - Inertia matrix.
    if (args.inertia is not None):
        nb_inertia = len(args.inertia)
        for j in range(0, nb_inertia):
            database.body[int(args.inertia[j][0]) - 1].activate_inertia()
            database.body[int(args.inertia[j][0]) - 1].inertia.mass = args.inertia[j][1]
            database.body[int(args.inertia[j][0]) - 1].inertia.I44 = args.inertia[j][2]
            database.body[int(args.inertia[j][0]) - 1].inertia.I55 = args.inertia[j][3]
            database.body[int(args.inertia[j][0]) - 1].inertia.I66 = args.inertia[j][4]
            database.body[int(args.inertia[j][0]) - 1].inertia.I45 = args.inertia[j][5]
            database.body[int(args.inertia[j][0]) - 1].inertia.I46 = args.inertia[j][6]
            database.body[int(args.inertia[j][0]) - 1].inertia.I56 = args.inertia[j][7]

    # Body - Inertia matrix.
    if (args.inertia_only is not None):
        nb_inertia = len(args.inertia_only)
        for j in range(0, inertia_only):
            database.body[int(args.inertia_only[j][0]) - 1].activate_inertia()
            database.body[int(args.inertia_only[j][0]) - 1].inertia.I44 = args.inertia_only[j][1]
            database.body[int(args.inertia_only[j][0]) - 1].inertia.I55 = args.inertia_only[j][2]
            database.body[int(args.inertia_only[j][0]) - 1].inertia.I66 = args.inertia_only[j][3]
            database.body[int(args.inertia_only[j][0]) - 1].inertia.I45 = args.inertia_only[j][4]
            database.body[int(args.inertia_only[j][0]) - 1].inertia.I46 = args.inertia_only[j][5]
            database.body[int(args.inertia_only[j][0]) - 1].inertia.I56 = args.inertia_only[j][6]

    # Body - Mass.
    if (args.mass is not None):
        nb_mass = len(args.mass)
        for j in range(0, nb_mass):
            database.body[int(args.mass[j][0]) - 1].activate_inertia()
            database.body[int(args.mass[j][0]) - 1].inertia.mass = args.mass[j][1]

    # Response Amplitude Operators.
    if(args.RAO is not None and Edition == 'EE'):
        RAO = ResponseAmplitudeOperator(database)
        RAO.computeRAO()

    # Mean wave drift loads.
    if (args.drift is not None and Edition == 'EE'):
        Drift = WaveDriftKochin(database)
        Drift.computeDriftForce()

    # Symmetry of the HDB.
    if(args.no_sym_hdb is False):
        database.symmetry_HDB()

    # Writing the hdb5 output file.
    if(args.write is not None):
        database.export_hdb5(args.write)

    # Plot added mass and damping coefficients.
    if (args.plot_radiation is not None):
        nb_plots_radiation = len(args.plot_radiation)
        for j in range(0, nb_plots_radiation):
            database.Plot_Radiation_coeff(ibody_force=int(args.plot_radiation[j][0]), iforce=int(args.plot_radiation[j][1]), ibody_motion=int(args.plot_radiation[j][2]),
                                          idof=int(args.plot_radiation[j][3]))

    # Plot diffraction loads.
    if (args.plot_diffraction is not None):
        nb_plots_diffraction = len(args.plot_diffraction)
        for j in range(0, nb_plots_diffraction):
            database.Plot_Diffraction(ibody=int(args.plot_diffraction[j][0]), iforce=int(args.plot_diffraction[j][1]), iwave=int(args.plot_diffraction[j][2]))

    # Plot Froude-Krylov loads.
    if (args.plot_froude_krylov is not None):
        nb_plots_froude_krylov = len(args.plot_froude_krylov)
        for j in range(0, nb_plots_froude_krylov):
            database.Plot_Froude_Krylov(ibody=int(args.plot_froude_krylov[j][0]), iforce=int(args.plot_froude_krylov[j][1]), iwave=int(args.plot_froude_krylov[j][2]))

    # Plot excitation loads.
    if (args.plot_excitation is not None):
        nb_plots_excitation = len(args.plot_excitation)
        for j in range(0, nb_plots_excitation):
            database.Plot_Excitation(ibody=int(args.plot_excitation[j][0]), iforce=int(args.plot_excitation[j][1]), iwave=int(args.plot_excitation[j][2]))

    # Plot impulse response functions.
    if (args.plot_irf is not None):
        nb_plots_irf = len(args.plot_irf)
        for j in range(0, nb_plots_irf):
            database.Plot_IRF(ibody_force=int(args.plot_irf[j][0]), iforce=int(args.plot_irf[j][1]), ibody_motion=int(args.plot_irf[j][2]),
                                          idof=int(args.plot_irf[j][3]))

    # Plot impulse response function with speed velocity.
    if (args.plot_irf_speed is not None):
        nb_plots_irf_speed = len(args.plot_irf_speed)
        for j in range(0, nb_plots_irf_speed):
            database.Plot_IRF_speed(ibody_force=int(args.plot_irf_speed[j][0]), iforce=int(args.plot_irf_speed[j][1]), ibody_motion=int(args.plot_irf_speed[j][2]),
                              idof=int(args.plot_irf_speed[j][3]))
if __name__ == '__main__':
    main()