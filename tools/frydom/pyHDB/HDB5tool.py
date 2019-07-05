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

from frydom.pyHDB.HDB5_v2 import *

try:
    import argcomplete

    acok = True
except:
    acok = False

def creation_parser_CE():

    parser = argparse.ArgumentParser(
        description="""  --  HDB5tool  --
            A Python module and a command line utility to add write HDB5 file from Nemoh output files.\n\n  Example of use:\n\n  hdb5tool --help""",
        formatter_class=argparse.RawDescriptionHelpFormatter)

    return parser

def get_parser(parser):

    # Path to Nemoh.cal.
    parser.add_argument('--path_to_nemoh_cal','-cal', action="store", nargs=1, metavar='Arg', help="""
                Path to the folder including the file Nemoh.cal.""")

    # Discretization - Wave directions.
    parser.add_argument('--discretization_waves','--discretization_wave','-dis_waves', '-dis_wave', '-dw', action="store", nargs=1, metavar='Arg', help="""
                Integer for the new discretization of the wave directions.""")

    # Discretization - Wave frequencies.
    parser.add_argument('--discretization_frequencies','--discretization_freq','--discretization_frequency','-dis_freq','-dis_frequency','-dis_frequencies','-df', action="store", nargs =1, metavar='Arg',help="""
                Integer for the new discretization of the wave frequencies.""")

    # Discretization - Final time.
    parser.add_argument('--final_time_irf','--final_time', '-ft', action="store", nargs = 1, metavar = 'Arg', help="""
                Final time for the computation of the impulse response functions.""")

    # Discretization - Time step.
    parser.add_argument('--time_step_irf', '--time_step', '-dt', action="store", nargs=1, metavar='Arg', help="""
                    Time step for the computation of the impulse response functions.""")

    # Body - Activate hydrostatics (useless).
    parser.add_argument('--activate_hydrostatics','--activate_hydrostatic', '-active_hs', '-activate_hs', nargs = '+', metavar = 'Arg', action="append",help="""
                Activate hydrostatics for the body of index given in argument.""")

    # Body - Hydrostatic matrix.
    parser.add_argument('--hydrostatics','--hydrostatic', '-hs', nargs=7, metavar=('id', 'k33', 'k44', 'k55', 'k34', 'k35', 'k45'), action="append",help="""
                Hydrostatic coefficients (K33, K44, K55, K34, K35, K45) for the body of index given in first argument.""")

    # Body - Activate inertia (useless).
    parser.add_argument('--activate_inertia', '-active_i', action="append",nargs='+', metavar='Arg',help="""
                Activate inertia for the body of index given in argument.""")

    # Body - Inertia matrix (inertias and mass).
    parser.add_argument('--inertia', '-i', nargs=8, metavar=('id', 'mass', 'i44', 'i55', 'i66', 'i45', 'i46', 'i56'), action="append",help="""
                Inertia coefficients and mass (Mass, I44, I55, I66, I45, I46, I56) for the body of index given in first argument.""")

    # Body - Inertia matrix (inertia only).
    parser.add_argument('--inertia_only', '-io', nargs=7, metavar=('id', 'i44', 'i55', 'i66', 'i45', 'i46', 'i56'), action="append",help="""
                Inertia coefficients only (I44, I55, I66, I45, I46, I56) for the body of index given in first argument.""")

    # Body - Mass.
    parser.add_argument('--mass', '--Mass', '-m', nargs=2, metavar=('id', 'mass'), action="append",help="""
                Mass of the body of index given in argument.""")

    # Filtering impulse response functions.
    parser.add_argument('--cutoff_irf','-co_irf', '-coirf', nargs = 5, metavar=('tc', 'ibody_force', 'iforce', 'ibody_motion', 'idof'), action="append",help="""
                Application of the filter with a cutoff time tc to the impulse response functions of ibody_force along iforce for a motion of ibody_motion along idof and plot the irf.""")

    # Filtering ALL impulse response functions.
    parser.add_argument('--cutoff_irf_all', '-co_irf_all', '-coirf_all', nargs=1, metavar=('tc'), action="store", help="""
                    Application of the filter with a cutoff time tc to ALL impulse response functions.""")

    # Filtering impulse response functions with forward speed.
    parser.add_argument('--cutoff_irf_speed','-co_irf_speed', '-coirf_speed', nargs = 5, metavar=('tc', 'ibody_force', 'iforce', 'ibody_motion', 'idof'), action="append",help="""
                Application of the filter with a cutoff time tc to the impulse response functions with forward speed of ibody_force along iforce for a motion of ibody_motion along idof and plot the irf.""")

    # Filtering ALL impulse response functions with forward speed.
    parser.add_argument('--cutoff_irf_all_speed', '-co_irf_all_speed', '-coirf_all_speed', nargs=1, metavar=('tc'), action="append", help="""
                    Application of the filter with a cutoff time tc to ALL impulse response functions with forward speed .""")

    # No symmetrization of the HDB.
    parser.add_argument('--sym_hdb','-sym', '-s', action="store_true",help="""
                Symmetrization of the HDB.""")

    # Writing the hdb5 output file.
    parser.add_argument('--write', '--export','-w', action="store",help="""
                Writing the hdb5 output file with the given name.""")

    # Plot added mass and damping coefficients.
    parser.add_argument('--plot_radiation', '--plots_radiation', '--plot_AB','-pab', '-prad', nargs = 4, metavar=('ibody_force', 'iforce', 'ibody_motion', 'idof'), action="append",help="""
                Plot the added mass and damping coefficients of ibody_force along iforce for a motion of ibody_motion along idof.""")

    # Plot diffraction loads.
    parser.add_argument('--plot_diffraction', '--plots_diffraction', '--plot_diff','-pd', '-pdiff', nargs = 3, metavar=('ibody', 'iforce', 'iwave'), action="append",help="""
                Plot the diffraction loads of ibody along iforce for iwave.""")

    # Plot Froude-Krylov loads.
    parser.add_argument('--plot_froude_krylov', '--plots_froude_krylov', '--plot_fk','-pfk', nargs = 3, metavar=('ibody', 'iforce', 'iwave'), action="append",help="""
                Plot the Froude-Krylov loads of ibody along iforce for iwave.""")

    # Plot excitation loads.
    parser.add_argument('--plot_excitation', '--plots_excitation', '--plot_exc','-pe', '-pexc', nargs = 3, metavar=('ibody', 'iforce', 'iwave'), action="append",help="""
                Plot the excitation loads of ibody along iforce for iwave.""")

    # Plot IRF.
    parser.add_argument('--plot_irf', '--plots_irf','-pirf', nargs = 4, metavar=('ibody_force', 'iforce', 'ibody_motion', 'idof'), action="append",help="""
                Plot the impulse response functions of ibody_force along iforce for a motion of ibody_motion along idof.""")

    # Plot IRF speed.
    parser.add_argument('--plot_irf_speed', '--plots_irf_speed', '--plot_irf_ku','-pirfs','-pirfku', '-pirf_speed', nargs = 4, metavar=('ibody_force', 'iforce', 'ibody_motion', 'idof'), action="append",help="""
                Plot the impulse response functions with speed velocity of ibody_force along iforce for a motion of ibody_motion along idof.""")

    # Reading a hdb5 file.
    parser.add_argument('--read','-r', action="store",help="""
                Reading a hdb5 file with the given name.""")

    # Initialization of the hdb.
    parser.add_argument('--initialization','-init', action="store_true",help="""
                Initialization of the hydrodynamic database: computation of the Froude-Krylov loads, IRF, etc.""")

    # Report generation (rst, html).
    parser.add_argument('--report_generation', '--report', '-rg', action="store", help="""
                Report generation about the hydrodynamic database in the defined folder.""")

    # Latex and pdf file generation.
    parser.add_argument('-latex', '-tex', '-pdf', '-latexpdf', action="store_true", help="""
                        latex and pdf file generation about the hydrodynamic database in the defined folder.""")

    return parser

def Read_cal_hdb5(args):

    # BEM reader.
    if (args.path_to_nemoh_cal is not None): # Nemoh.
        database = HDB5()
        database.nemoh_reader(args.path_to_nemoh_cal[0])
        database._pyHDB.solver = "Nemoh"

    # Reading a hdb5 file.
    if (args.read is not None):
        database = HDB5()
        database.read_hdb5(args.read)
        database._is_initialized = True  # No initialization except if asked.

    if (args.path_to_nemoh_cal is None and args.read is None):
        print("No input file has been defined.")
        print("Please give a Nemoh.cal file (-cal) or a .hdb5 file (-r) as input.")
        exit()
    elif (args.path_to_nemoh_cal is not None and args.read is not None):
        print("Only one input file may be defined.")
        print("Please choose between given a Nemoh.cal file (-cal) and a .hdb5 file (-r) as input.")
        exit()
    else:
        has_single_input = True

    return database

def get_Arg_part_1_CE(args, database):

    # Discretization - Wave directions.
    if (args.discretization_waves is not None):
        if(int(args.discretization_waves[0]) <= 1):
            print("The number of the wave direction discretization must be higher or equal to 2.")
            exit()
        database.discretization.nb_wave_directions = int(args.discretization_waves[0])

    # Discretization - Wave frequencies.
    if (args.discretization_frequencies is not None):
        if (int(args.discretization_frequencies[0]) <= 1):
            print("The number of the wave frequency discretization must be higher or equal to 2.")
            exit()
        database.discretization.nb_frequencies = int(args.discretization_frequencies[0])

    # Discretization - Final time for IRF.
    if (args.final_time_irf is not None):
        database.discretization._final_time = float(args.final_time_irf[0])

    # Discretization - Time step for IRF.
    if (args.time_step_irf is not None):
        database.discretization._delta_time = float(args.time_step_irf[0])

    # Initialize pyHDB.
    if (args.path_to_nemoh_cal is not None or args.initialization is True):  # _initialize is automatically called when a .cal is read.
        database._initialize()

    # Body - Active hydrostatics (useless).
    if (args.activate_hydrostatics is not None):
        nb_activation_hydrostatics = len(args.activate_hydrostatics)
        for id in range(0, nb_activation_hydrostatics):
            database.body[int(args.activate_hydrostatics[id]) - 1].activate_hydrostatic()

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
        nb_inertia_only = len(args.inertia_only)
        for j in range(0, nb_inertia_only):
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

    # Filtering impulse response functions.
    if (args.cutoff_irf is not None):
        nb_cut_off_irf = len(args.cutoff_irf)
        for j in range(0, nb_cut_off_irf):
            database.Cutoff_scaling_IRF(tc=float(args.cutoff_irf[j][0]), ibody_force=int(args.cutoff_irf[j][1]), iforce=int(args.cutoff_irf[j][2]),
                                        ibody_motion=int(args.cutoff_irf[j][3]), idof=int(args.cutoff_irf[j][4]))

    # Filtering ALL impulse response functions.
    if (args.cutoff_irf_all is not None):
        for body_force in database.body:
            for iforce in range(0,6):
                for body_motion in database.body:
                    for idof in range(0,6):
                        database.Cutoff_scaling_IRF(tc=float(args.cutoff_irf_all[0]), ibody_force=body_force.i_body, iforce=iforce,
                                                            ibody_motion=body_motion.i_body, idof=idof, auto_apply = True)

    # Filtering impulse response functions with forward speed.
    if (args.cutoff_irf_speed is not None):
        nb_cut_off_irf_speed = len(args.cutoff_irf_speed)
        for j in range(0, nb_cut_off_irf_speed):
            database.Cutoff_scaling_IRF_speed(tc=float(args.cutoff_irf_speed[j][0]), ibody_force=int(args.cutoff_irf_speed[j][1]), iforce=int(args.cutoff_irf_speed[j][2]),
                                              ibody_motion=int(args.cutoff_irf_speed[j][3]), idof=int(args.cutoff_irf_speed[j][4]))

    # Filtering ALL impulse response functions with forward speed.
    if (args.cutoff_irf_all_speed is not None):
        for body_force in database.body:
            for iforce in range(0, 6):
                for body_motion in database.body:
                    for idof in range(0, 6):
                        database.Cutoff_scaling_IRF_speed(tc=float(args.cutoff_irf_all[0]), ibody_force=body_force.i_body, iforce=iforce,
                                                    ibody_motion=body_motion.i_body, idof=idof, auto_apply=True)

    return database

def get_Arg_part_2_CE(args, database):

    # Symmetry of the HDB.
    if (args.sym_hdb is True):
        database.symmetry_HDB()

    return database

def get_Arg_part_3_CE(args, database):

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

    return database

def get_Arg_part_4_CE(args, database):

    # Writing the hdb5 output file.
    if (args.write is not None):
        database.export_hdb5(args.write)

    # Report generation - hdb only.
    if (args.report_generation is not None):
        database.report_writing_HDB(args.report_generation)

    return database

def get_Arg_part_5_CE(args, database):

    # Report generation - Writing index.rst.
    if (args.report_generation is not None):
        database.report_writing_Index(args.report_generation)

    # Report generation - Building the html file.
    if (args.report_generation is not None):
        database.report_building_html(args.report_generation)

    # Report generation - Building both the latex and pdf files.
    if (args.report_generation is not None and args.latex is True):
        database.report_building_pdf(args.report_generation)

    return database

def main():

    ####################################################################################################################
    #                                                   Parser
    ####################################################################################################################

    parser = creation_parser_CE()
    parser = get_parser(parser)

    if acok:
        argcomplete.autocomplete(parser)

    args, unknown = parser.parse_known_args()

    ####################################################################################################################
    #                               Selection of an input file: Nemoh file or hdb5 file
    ####################################################################################################################

    database = Read_cal_hdb5(args)

    ####################################################################################################################
    #                                               Reading arguments
    ####################################################################################################################

    # 1st set of arguments - FryDoM CE.
    database = get_Arg_part_1_CE(args, database)

    # 2nd set of arguments - FRyDoM CE.
    database = get_Arg_part_2_CE(args, database)

    # 3rd set of arguments - FRyDoM CE.
    database = get_Arg_part_3_CE(args, database)

    # 4th set of arguments - FRyDoM CE.
    database = get_Arg_part_4_CE(args, database)

    # 5th set of arguments - FRyDoM CE.
    database = get_Arg_part_5_CE(args, database)

if __name__ == '__main__':
    main()