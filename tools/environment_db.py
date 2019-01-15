#!/usr/bin/env python
#  -*- coding: utf-8 -*-
"""Module to create a environment database for frydom hydrodynamic database"""

import numpy as np
from datetime import datetime


class EnvironmentDB(object):

    def __init__(self):
        self._gravity = None
        self._nb_body = None
        self._normalization_length = None
        self._water_density = None
        self._water_depth = None

    @property
    def gravity(self):
        return self._gravity

    @property
    def nb_body(self):
        return self._nb_body

    @property
    def water_depth(self):
        return self._water_depth

    @water_depth.setter
    def water_depth(self, value):
        if value == 'infinite':
            value = 0
        self._water_depth = value

    @property
    def normalization_length(self):
        return self._normalization_length

    @normalization_length.setter
    def normalization_length(self, value):
        if value > 0:
            self._normalization_length = value
        else:
            print("warning : value must be > 0")

    @property
    def water_density(self):
        return self._water_density

    @water_density.setter
    def water_density(self, value):
        if value > 0:
            self._water_density = value
        else:
            print("warning : value must be > 0")

    def load_data(self, hdb):

        self._gravity = hdb.grav
        self._nb_body = hdb.body_mapper.nb_bodies
        self.normalization_length = 1.
        self.water_depth = hdb.depth
        self.water_density = hdb.rho_water

    def write_hdb5(self, writer):

        # Date
        dset = writer.create_dataset('CreationDate', data=str(datetime.now()))
        dset.attrs['Description'] = "Date of the creation of this database"

        # Gravity acceleration
        dset = writer.create_dataset('GravityAcc', data=self.gravity)
        dset.attrs['Unit'] = 'm/s**2'
        dset.attrs['Description'] = "Gravity acceleration"

        # Water density
        dset = writer.create_dataset('WaterDensity', data=self.water_density)
        dset.attrs['Unit'] = 'kg/m**3'
        dset.attrs['Description'] = 'Water Density'

        # Normalisation length
        dset = writer.create_dataset('NormalizationLength', data=self.normalization_length)
        dset.attrs['Unit'] = 'm'
        dset.attrs['Description'] = 'Normalization length'

        # Water height
        dset = writer.create_dataset('WaterDepth', data=self.water_depth)
        dset.attrs['Unit'] = 'm'
        dset.attrs['Description'] = 'Water depth. It is 0. for infinite values and positive for finite values'

        # Number of interacting bodies
        dset = writer.create_dataset('NbBody', data=self.nb_body)
        dset.attrs['Description'] = 'Number of interacting bodies'