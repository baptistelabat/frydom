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

# Path to Nemoh.cal.
parser.add_argument('--path_to_nemoh_cal','-cal', action="store",help="""
            Path to the folder including the file Nemoh.cal.""")

# Discretization - Wave directions.
parser.add_argument('--discretization_waves','--discretization_wave','-dis_waves', '-dis_wave', action="store",help="""
            Integer for the new discretization of the wave directions.""")

# Discretization - Wave frequencies.
parser.add_argument('--discretization_frequencies','--discretization_freq','--discretization_frequency','-dis_freq','-dis_frequency','-dis_frequencies', action="store",help="""
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
parser.add_argument('--inertia', '-i', nargs=7, metavar=('id', 'i33', 'i44', 'i55', 'i34', 'i35', 'i45'), action="append",help="""
            Inertia coefficients (K33, K44, K55, K34, K35, K45) for the body of index given in first argument.""")

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

def main():

    ####################################################################################################################
    #                                               Argparse.
    ####################################################################################################################

    if acok:
        argcomplete.autocomplete(parser)

    args, unknown = parser.parse_known_args()

    # Edition.
    if (args.edition != None):
        Edition = args.edition
        if(Edition == 'CE' or Edition == 'GPL' or Edition == 'Community' or Edition == 'community' or Edition == 'ce' or Edition == 'gpl'):
            Edition = 'CE'
        elif(Edition == 'EE' or Edition == 'Entreprise' or Edition == 'entreprise' or Edition == 'ee'):
            Edition = 'EE'
        else:
            print("The edition given in input is not known, please choose between CE or EE.")
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
            database.body[int(args.inertia[j][0]) - 1].inertia.I44 = args.inertia[j][1]
            database.body[int(args.inertia[j][0]) - 1].inertia.I55 = args.inertia[j][2]
            database.body[int(args.inertia[j][0]) - 1].inertia.I66 = args.inertia[j][3]
            database.body[int(args.inertia[j][0]) - 1].inertia.I45 = args.inertia[j][4]
            database.body[int(args.inertia[j][0]) - 1].inertia.I46 = args.inertia[j][5]
            database.body[int(args.inertia[j][0]) - 1].inertia.I56 = args.inertia[j][6]

    # Body - Mass.
    if (args.mass is not None):
        nb_mass = len(args.mass)
        for j in range(0, nb_mass):
            database.body[int(args.inertia[j][0]) - 1].activate_inertia()
            database.body[int(args.inertia[j][0]) - 1].inertia.mass = args.mass[j][1]

    # Response Amplitude Operators.
    if(args.RAO is not None and Edition == 'EE'):
        RAO = ResponseAmplitudeOperator(database)
        RAO.computeRAO()

    # Mean wave drift loads.
    if (args.drift is not None and Edition == 'EE'):
        Drift = WaveDriftKochin(database)
        Drift.computeDriftForce()

    # Symmetry of the HDB.
    if(args.no_sym_hdb is None):
        database.symmetry_HDB()

    # Writing the hdb5 output file.
    if(args.write is not None):
        database.export_hdb5(args.write)

if __name__ == '__main__':
    main()