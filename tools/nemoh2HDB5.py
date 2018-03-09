#!/usr/bin/env python
#  -*- coding: utf-8 -*-

"""A command line utility to convert Nemoh results into a HDB5 file"""

import os
from HydroDB.bem_reader import NemohReader
from HydroDB.hdb5_writer import write_hdb5


def main():
    import argparse

    parser = argparse.ArgumentParser(
        description="""  --- nemoh2HDB5 ---
        Command line utility to convert Nemoh results into a HDB5 file to be used by FRyDoM.
                    
        """,
        epilog='-- FRyDoM framework.\nNemoh2HDB5 Utility',
        formatter_class=argparse.RawDescriptionHelpFormatter
    )

    parser.add_argument('IDIR', help='Path of the directory that contains the Nemoh.cal file', type=str)
    parser.add_argument('-o', '--output', help='Path to the output HDB5 file', type=str)
    parser.add_argument('-n', '--nb-faces-by-wavelength',
                        help='Specify the number of faces by wavelength that BEM computations should take into account',
                        type=int)

    # Parsing command line
    args, unknown = parser.parse_known_args()

    input_directory = args.IDIR

    if not os.path.isabs(input_directory):
        input_directory = os.path.abspath(args.IDIR)

    # Verifying there is the Nemoh.cal file inside input_directory
    nemoh_cal_file = os.path.join(input_directory, 'Nemoh.cal')
    if not os.path.isfile(nemoh_cal_file):
        raise AssertionError('Folder %s seems not to be a Nemoh calculation folder as '
                             'we did not find Nemoh.cal' % input_directory)

    print('========================')
    print('Reading Nemoh results...')
    print('========================')

    nb_faces_by_wavelength = args.nb_faces_by_wavelength

    if nb_faces_by_wavelength is None:
        nb_faces_by_wavelength = 10

    reader = NemohReader(cal_file=nemoh_cal_file, test=True, nb_face_by_wave_length=nb_faces_by_wavelength)
    print('-------> Nemoh data successfully loaded from "%s"' % input_directory)

    print('========================')
    print('Writing HDB5 database...')
    print('========================')

    output_file = args.output

    if output_file is None:
        hdb5_file = os.path.abspath('frydom.hdb5')

    else:
        # Verifying that the output file has the extension .hdb5
        root, ext = os.path.splitext(output_file)
        if not ext == '.hdb5':
            raise IOError('Please register the output file with a .hdb5 extension')

        hdb5_file = output_file

        if not os.path.isabs(output_file):
            hdb5_file = os.path.abspath(hdb5_file)

    try:
        write_hdb5(reader.hydro_db, out_file=hdb5_file)
    except IOError:
        raise IOError('Problem in writing HDB5 file at location %s' % hdb5_file)


    print('-------> "%s" has been written' % hdb5_file)

if __name__ == '__main__':
    main()
