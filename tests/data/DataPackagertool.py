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
    ArgParse module of DataPackager
"""

import os
import argparse

import DataPackager

try:
    import argcomplete
    acok = True
except:
    acok = False


def creation_parser():

    parser = argparse.ArgumentParser(
        description=
        """ -- Data Packager -- 
            A python module and command line utility to manage data archive for FRyDoM on Amazon S3 \n\n
            Example of use: \n\n
            DataPackagerTool --help
        """,
        formatter_class=argparse.RawDescriptionHelpFormatter
    )
    return parser


def get_parser(parser):

    parser.add_argument('--add', '-a', nargs='+',
                        help='List of directories or files to be added to the archive')

    parser.add_argument('--type_revision', '-tr', nargs=1, metavar='type_revision',
                        help='Type of revision to be applied (release/major/minor)')

    parser.add_argument('-no_upload', metavar="upload_data", action="store_false",
                        help='Deactivate data upload to Amazon S3')

    parser.add_argument('--diff', '-d', metavar="get_diff", action="store_true",
                        help="Show difference with local data")

    parser.add_argument('--remote_version', '-rv', nargs=1, metavar='remote_version',
                        help='Specify the version of the data loaded from Amazon S3')

    parser.add_argument('--new_version', '-nv', nargs=1, metavar='new_version',
                        help='Specify the version of the new archive to be created')

    parser.add_argumment('--remove', '-r', nargs='+',
                         help='Remove directories or files from the data archive')

    return parser


def main():

    # Set parser

    parser = create_parser()
    parser = get_parser(parser)

    if acok:
        argcomplete.autocomplete(parser)

    args, unknown = parser.parse_known_args()

    package = DataPackager.Packager()

    if args.add is not None:
        for element in args.add:
            package.add(element)

    package.update()

    #if args.upload_data:
    #    package.upload()




if __name__ == '__main__':
    main()