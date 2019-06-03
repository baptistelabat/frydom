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
    Module to create a hydrodynamic database for FRyDoM.
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
    if(args.path_to_nemoh_cal != None):
        database = HDB5()
        database.nemoh_reader(args.path_to_nemoh_cal)
    else:
        print("The path to folder including the file Nemoh.cal is required.")






if __name__ == '__main__':
    main()