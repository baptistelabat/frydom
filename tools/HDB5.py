#!/usr/bin/env python
#  -*- coding: utf-8 -*-
"""Module to create a hydrodynamic database for frydom"""

import os, h5py

from HydroDB.bem_reader import NemohReader
from HydroDB.body_db import BodyDB

from environment_db import EnvironmentDB
from discretization_db import DiscretizationDB


class HDB5(object):

    def __init__(self):

        self._hdb = None
        self._environment = EnvironmentDB()
        self._discretization = DiscretizationDB()
        self._bodies = []

        return

    def nemoh_reader(self, input_directory='.', nb_faces_by_wavelength=None):

        if not os.path.isabs(input_directory):
            input_directory = os.path.abspath(input_directory)

        # Verifying there is the Nemoh.cal file inside input_directory
        nemoh_cal_file = os.path.join(input_directory, 'Nemoh.cal')
        if not os.path.isfile(nemoh_cal_file):
            raise AssertionError('Folder %s seems not to be a Nemoh calculation folder as '
                                 'we did not find Nemoh.cal' % input_directory)

        print('========================')
        print('Reading Nemoh results...')
        print('========================')

        if nb_faces_by_wavelength is None:
            nb_faces_by_wavelength = 10

        reader = NemohReader(cal_file=nemoh_cal_file, test=True, nb_face_by_wave_length=nb_faces_by_wavelength)

        # Save the whole database
        self._hdb = reader.hydro_db

        # Save the environment
        self._environment.load_data(reader.hydro_db)

        # Save the discretization
        self._discretization.load_data(reader.hydro_db)

        # Load bodies
        for i_body in range(reader.hydro_db.body_mapper.nb_bodies):
            self._bodies.append(BodyDB(reader.hydro_db, i_body))

        print('-------> Nemoh data successfully loaded from "%s"' % input_directory)

    def get_body_list(self):
        for body in self._bodies:
            print("body id: %i, name : %s" % (body.id, body.name))
        return

    def get_body(self, id=None, name=None):

        if name:
            return self._find_body_by_name(name)
        elif id:
            return self._bodies[id]
        else:
            print("warning : id or name must be defined")

        return None

    @property
    def body(self):
        return self._bodies

    def _find_body_by_name(self, name):

        for body in self._bodies:
            if body.mesh.name == name:
                return body

        print("warning : no body found with name %s" % name)
        return None

    def initialize(self):

        for body in self._bodies:

            if body.wave_drift:
                body.wave_drift.discrete_wave_dir = self._discretization.wave_dirs
                body.wave_drift.initialize()

        return



    def write_hdb5(self, output_file=None):

        print('========================')
        print('Writing HDB5 database...')
        print('========================')

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

            with h5py.File(hdb5_file, 'w') as writer:
                self._environment.write_hdb5(writer)
                self._discretization.write_hdb5(writer)
                for body in self._bodies:
                    body.write_hdb5(writer)

        except IOError:
            raise IOError('Problem in writing HDB5 file at location %s' % hdb5_file)

        print('-------> "%s" has been written' % hdb5_file)








