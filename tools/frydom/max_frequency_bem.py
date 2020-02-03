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
This command line utility purpose is to calculate the maximum frequency that you can ask to BEM software for a given
mesh refinement. A common practice is to get 10 faces by wavelength to be converged with mesh refinement. The wavelength
considered is the minimum wave length of the wave spectrum which is linked to frequency through the wave dispersion
relation.
"""
import argparse
import os
from math import pi, fabs, tanh, sqrt

from meshmagick import mmio
from meshmagick.mesh import Mesh


def main():
    parser = argparse.ArgumentParser(
        description="""--- max_frequency_bem ---
        Command line utility to get the maximum frequency allowed by a given mesh discretization for the BEM 
        computations to be accurate.
        
        It relies on meshmagick tool for mesh reading
        
        """,
        epilog='-- FRyDoM framework.\nmax_frequency_bem utility',
        formatter_class=argparse.RawDescriptionHelpFormatter
    )

    parser.add_argument('infilename',  # TODO : voir pour un typ=file pour tester l'existence
                        help='path of the input mesh file in any supported format', type=str)

    parser.add_argument('-ifmt', '--input-format',
                        help="""Input format. Meshmagick will read the input file considering the
                         INPUT_FORMAT rather than using the extension
                         """, type=str)

    parser.add_argument('-n', '--nb-faces-by-wavelength',
                        help='Specify the number of faces by wavelength that BEM computations should take into account',
                        type=int)

    parser.add_argument('-g', '--gravity', help='Acceleration of gravity in m/s**2. Default is 9.81',
                        type=float, default=9.81)

    parser.add_argument('-wd', '--water-depth', help='Water depth in meters. Default is infinite (put 0 explicitely)',
                        type=float, default=0.)

    args, unknown = parser.parse_known_args()

    if args.input_format is not None:
        ifmt = args.input_format
    else:
        _, ext = os.path.splitext(args.infilename)
        ifmt = ext[1:].lower()
        if ifmt == '':
            raise IOError('Unable to determine the input file format from its extension. '
                          'Please specify an input format.')

    # Loading mesh from file
    if os.path.isfile(args.infilename):
        vertices, faces = mmio.load_mesh(args.infilename, ifmt)
        mesh = Mesh(vertices, faces)
    else:
        raise IOError('file %s not found' % args.infilename)

    # Number of faces by wave length necessary to keep accuracy good in BEM computations
    nb_faces_by_wavelength = args.nb_faces_by_wavelength

    if nb_faces_by_wavelength is None:
        nb_faces_by_wavelength = 10

    max_edge_length = mesh.max_edge_length

    wave_length = nb_faces_by_wavelength * max_edge_length
    wave_number = 2. * pi / wave_length

    w2 = args.gravity * wave_number

    depth = fabs(args.water_depth)
    if depth != 0.:
        w2 *= tanh(wave_number * depth)

    w = sqrt(w2)

    print('\nBased on the rule of %u faces by wavelength, the specified mesh with maximum edge length of %.3f m \n'
          'allows to perform BEM computations until the frequency:\n' % (nb_faces_by_wavelength, max_edge_length))

    print('\t\t*****************************')
    print('\t\t***   Wmax = %.2f rad/s   ***' % w)
    print('\t\t*****************************\n')

    print('with:')

    print('\t+ Gravity:      %.3f m/s**2' % args.gravity)

    if depth == 0.:
        print('\t+ Water depth: INFINITE')
    else:
        print('\t+ Water depth:  %.3f m (FINITE)' % depth)

    print('\n')


if __name__ == '__main__':
    main()
