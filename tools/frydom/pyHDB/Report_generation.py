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
import copy
import numpy as np

import meshmagick.MMviewer

class report():
    """
        Class for defining and writing a report.
    """

    def __init__(self, output_folder):

        # Index file.
        self._RstIndex = RstCloth()
        self._IndexFileName = "index" # If you change this name, you need to update conf.py.

        # Input parameters file.
        self._RstInputParam = RstCloth()
        self._InputParamFileName = "Input_parameters"

        # conf.py file for generating the html file.
        my_path = os.path.abspath(os.path.dirname(__file__))
        self._conf_file = my_path+"/conf.py" # In the frydom-CE deposit.

        # _static folder.
        self.static_folder = os.path.join(output_folder,'_static/')
        if not os.path.exists(self.static_folder):
            os.makedirs(self.static_folder)

        # Source folder.
        self.source_folder = os.path.join(output_folder, 'Source/')
        if not os.path.exists(self.source_folder):
            os.makedirs(self.source_folder)

    def WriteIndex(self):
        """This function writes the rst file Index.rst."""

        self._RstIndex.title("Hydrodynamic database results")
        self._RstIndex.newline()
        self._RstIndex._add("This report presents the results of the hydrodynamic database obtained with **Nemoh**."
                           "It remains the input parameters of used in **Nemoh** and the post-processing achieved by **HDB5tool**.")
        self._RstIndex.newline()
        self._RstIndex.directive(name="toctree", fields=[('maxdepth', '3')])
        self._RstIndex.newline()
        self._RstIndex._add(".. Contents:")
        self._RstIndex.newline()
        self._RstIndex.h1("Input parameters for the frequency-domain solver")
        self._RstIndex.newline()
        self._RstIndex.directive(name="toctree", fields=[('maxdepth', '3')])
        self._RstIndex.newline()
        self._RstIndex.content('Source/'+self._InputParamFileName, indent=3, block='ct2')
        self._RstIndex.newline()
        # self._RstIndex.h1("Post-processing parameters")
        # self._RstIndex.newline()
        # self._RstIndex.directive(name="toctree", fields=[('maxdepth', '3')])
        # self._RstIndex.newline()
        # self._RstIndex.h1("Hydrodynamic database results")
        # self._RstIndex.newline()
        # self._RstIndex.directive(name="toctree", fields=[('maxdepth', '3')])
        # self._RstIndex.newline()
        # self._RstIndex.h2("Added mass coefficients")
        # self._RstIndex.newline()
        # self._RstIndex.h2("Damping coefficients")
        # self._RstIndex.newline()
        # self._RstIndex.h2("Diffraction loads")
        # self._RstIndex.newline()
        # self._RstIndex.h2("Froude-Krylov loads")
        # self._RstIndex.newline()
        # self._RstIndex.h2("Impulse response functions")
        self._RstIndex.h1("Indices and tables")
        self._RstIndex.newline()
        self._RstIndex.li([':ref:`genindex`'], bullet='*', block='li2')
        self._RstIndex.li([':ref:`modindex`'], bullet='*', block='li2')
        self._RstIndex.li([':ref:`search`'], bullet='*', block='li2')

    def WriteInputParameters(self, pyHDB, output_folder):
        """This function writes the rst file Input_parameters.rst."""

        self._RstInputParam.title("Input parameters")
        self._RstInputParam.newline()
        self._RstInputParam._add("This chapter lists the input parameters used in the frequency-domain linear potential flow based solver **Nemoh**."
                                 " They are listed in the next table:")
        self._RstInputParam.newline()
        self._RstInputParam._add('========================== ==================================')
        self._RstInputParam._add('Parameter                  Value')
        self._RstInputParam._add('========================== ==================================')
        self._RstInputParam._add('Water density              ' + str(pyHDB.rho_water)+' kg/m3')
        self._RstInputParam._add('Gravity constant           ' + str(pyHDB.grav) + ' m/s2')
        if(pyHDB.depth == float('inf')):
            self._RstInputParam._add('Water depth                Infinity')
        else:
            self._RstInputParam._add('Water depth            ' + str(pyHDB.depth) + ' m')
        self._RstInputParam._add('Number of bodies           ' + str(pyHDB.nb_bodies))
        self._RstInputParam._add('Number of wave frequencies ' + str(pyHDB.nb_wave_freq))
        self._RstInputParam._add('Number of wave directions  ' + str(pyHDB.nb_wave_dir))
        self._RstInputParam._add('========================== ==================================')
        self._RstInputParam.newline()

        self._RstInputParam._add("The main parameters of the bodies are listed below.")
        self._RstInputParam.newline()
        for body in pyHDB.bodies:
            self._RstInputParam._add('========================== ==================================')
            self._RstInputParam._add('Parameter                  Value')
            self._RstInputParam._add('========================== ==================================')
            self._RstInputParam._add('Body index                 ' + str(body.i_body))
            self._RstInputParam._add('Mesh                       ' + str(body.mesh.name))
            self._RstInputParam._add('Number of vertices         ' + str(body.mesh.nb_vertices))
            self._RstInputParam._add('Number of faces            ' + str(body.mesh.nb_faces))
            self._RstInputParam._add('Motion mask                ' + str(body.Motion_mask))
            self._RstInputParam._add('Force mask                 ' + str(body.Force_mask))
            self._RstInputParam._add('========================== ==================================')
            self._RstInputParam.newline()

            # Picture of the mesh.
            self.PlotMesh(body, output_folder, "Mesh_"+str(body.i_body)+".png")
            exit()

    def PlotMesh(self, body, output_folder, mesh_file):
        """This function plots and saves a picture of the body mesh."""

        # Rotation of a copy of the mesh.
        mesh_vizu = copy.deepcopy(body.mesh)
        mesh_vizu.rotate([-np.pi/3,-np.pi/16,-np.pi/8])

        # Showing the mesh.
        vtk_polydata = mesh_vizu._vtk_polydata()
        mesh_vizu.viewer = meshmagick.MMviewer.MMViewer()
        mesh_vizu.viewer.add_polydata(vtk_polydata)
        mesh_vizu.viewer.renderer.ResetCamera()
        mesh_vizu.viewer.render_window.Render()

        # Screenshot.
        mesh_vizu.viewer.screenshot()

        # Copy of the mesh in the _Static folder.
        copyfile("screenshot.png", self.static_folder + mesh_file) # "screenshot.png" is hard codded in the script MMviewer.py of Meshmagick.

        # Deletion of the screenshot.
        cwd = os.getcwd()
        os.remove(cwd + "/screenshot.png")

    def WriteRst(self, output_folder):
        """This functions writes a rst file."""

        Ext = '.rst'
        self._RstIndex.write(os.path.join(output_folder, self._IndexFileName + Ext)) # Index.rst.
        self._RstInputParam.write(os.path.join(self.source_folder, self._InputParamFileName + Ext)) # Input_parameters.rst.

    def BuildHTML(self, output_folder):
        """This function builds the html file from the rst files."""

        conf_file = os.path.join(output_folder, 'conf.py')
        copyfile(self._conf_file, conf_file)
        build_folder = os.path.join(output_folder, 'build/')
        os.system('sphinx-build -b html '+ str(output_folder) + ' ' + str(build_folder))

    def OpenHTML(self, output_folder):
        """This function opens the html file in a web browser."""

        html_file = os.path.join(output_folder,'build/',self._IndexFileName+'.html')
        webbrowser.open(html_file)