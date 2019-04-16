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
"""Module to create a environment database for frydom hydrodynamic database"""

import numpy as np
from datetime import datetime


class EnvironmentDB(object):

    """Class for dealing with environmental data."""

    def __init__(self):

        """
        Constructor of the class EnvironmentDB.
        """

        self._gravity = None
        self._nb_body = None
        self._normalization_length = None
        self._water_density = None
        self._water_depth = None

    @property
    def gravity(self):

        """This subroutine gives the gravity.

        Returns
        -------
        float
            Gravity.
        """

        return self._gravity

    @property
    def nb_body(self):

        """This subroutine gives the number of bodies.

        Returns
        -------
        int
            Number of bodies.
        """

        return self._nb_body

    @property
    def water_depth(self):

        """This subroutine gives the water depth.

        Returns
        -------
        float
            Water depth.
        """

        return self._water_depth

    @water_depth.setter
    def water_depth(self, value):

        """This subroutine sets the water depth.

        Parameter
        ----------
        Float : value
            Water depth.
        """

        if value == 'infinite':
            value = 0
        self._water_depth = value

    @property
    def normalization_length(self):

        """This subroutine gives the normalization length.

        Returns
        -------
        float
            Normalization length.
        """

        return self._normalization_length

    @normalization_length.setter
    def normalization_length(self, value):

        """This subroutine sets the normalization length.

        Parameter
        ----------
        Float : value
            Normalization length.
        """

        if value > 0:
            self._normalization_length = value
        else:
            print("warning : value must be > 0")

    @property
    def water_density(self):

        """This subroutine gives the water density.

        Returns
        -------
        float
            Water density.
        """

        return self._water_density

    @water_density.setter
    def water_density(self, value):

        """This subroutine sets the water density.

        Parameter
        ----------
        Float : value
            Water density.
        """

        if value > 0:
            self._water_density = value
        else:
            print("warning : value must be > 0")

    def load_data(self, hdb):

        """This subroutine loads the environmental data from the hydrodynamic database.

        Parameter
        ----------
        HydroDB : value
            Hydrodynamic database.
        """

        self._gravity = hdb.grav
        self._nb_body = hdb.body_mapper.nb_bodies
        self.normalization_length = 1.
        self.water_depth = hdb.depth
        self.water_density = hdb.rho_water

    def write_hdb5(self, writer):

        """This subroutine writes the environmental data into the *.hdb5 file.

        Parameters
        ----------
        Writer : string
            *.hdb5 file.
        """

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