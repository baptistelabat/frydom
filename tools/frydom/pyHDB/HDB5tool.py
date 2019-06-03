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

parser.add_argument('--edition','-ed', action="store",help="""
            Edition of FRyDoM: CE or ce for FRyDoM-CE, EE or ee for FRyDoM-EE.""")

def main():

    if acok:
        argcomplete.autocomplete(parser)

    args, unknown = parser.parse_known_args()

    # Edition.
    if (args.edition != None):
        Edition = args.edition
        if(Edition == 'CE' or Edition == 'GPL' or Edition == 'Community' or Edition == 'community' or Edition == 'ce' or Edition == 'gpl'):
            Edition = 'ce'
        elif(Edition == 'EE' or Edition == 'Entreprise' or Edition == 'entreprise' or Edition == 'ee'):
            Edition = 'ee'
        else:
            print("The edition given in input is not known, please choose between CE or EE.")
    else:
        print("The edition of FRyDoM is required. Please use --edition or -ed.")
        exit()




if __name__ == '__main__':
    main()