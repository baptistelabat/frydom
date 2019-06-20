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
    Module to generate a report about the hydrodynamic database.
"""

import os
from rstcloth.rstcloth import RstCloth
import webbrowser
from shutil import copyfile

class report():
    """
        Class for defining and writing a report.
    """

    def __init__(self, output_folder):

        # Rst file.
        self._RstData = RstCloth()

        # conf.py file for generating the html file.
        my_path = os.path.abspath(os.path.dirname(__file__))
        self._conf_file = my_path+"/conf.py" # In the frydom-CE deposit.

        # _static folder.
        static_folder = os.path.join(output_folder,'_static/')
        if not os.path.exists(static_folder):
            os.makedirs(static_folder)

    def SetRstTitle(self, title="Hydrodynamic database results"):
        """This function sets the title of the rst file."""

        self._RstData.title(title)
        self._RstData.newline()

    def WriteRst(self, output_folder, filename):
        """This functions writes a rst file."""

        path = os.path.join(output_folder, filename)
        self._RstData.write(path)

    def BuildHTML(self, output_folder):
        """This function builds the html file from the rst files."""

        conf_file = os.path.join(output_folder, 'conf.py')
        copyfile(self._conf_file, conf_file)
        build_folder = os.path.join(output_folder, 'build/')
        os.system('sphinx-build -b html '+ str(output_folder) + ' ' + str(build_folder))

    def OpenHTML(self, output_folder, filename):
        """This function opens the html file in a web browser."""

        html_file = os.path.join(output_folder,'build/',filename+'.html')
        webbrowser.open(html_file)