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
import datetime
from scipy import interpolate

import meshmagick.MMviewer
from plot_db import *
from conf import latex_documents, latex_logo

class report():
    """
        Class for defining and writing a report.
    """

    def __init__(self, output_folder):

        # Extension.
        self.Ext = '.rst'

        # Index file.
        self._RstIndex = RstCloth()
        self._IndexFileName = "index" # If you change this name, you need to update conf.py.

        # Convention file.
        self._RstConventions = RstCloth()
        self._ConventionsFileName = "Conventions"

        # Input parameters file.
        self._RstInputParam = RstCloth()
        self._InputParamFileName = "Input_parameters"

        # Added mass and damping coefficients.
        self._RstAddedMass = RstCloth()
        self._AddedMassFileName = "Added_mass_Damping"

        # Diffraction loads.
        self._RstDiffraction = RstCloth()
        self._DiffractionFileName = "Diffraction"

        # Froude-Krylov loads.
        self._RstFroudeKrylov = RstCloth()
        self._FroudeKrylovFileName = "Froude_Krylov"

        # Excitation loads.
        self._RstExcitation = RstCloth()
        self._ExcitationFileName = "Excitation"

        # Infinite added mass matrix.
        self._RstInfAddedMass = RstCloth()
        self._InfAddedMassFileName = "Inf_Added_mass"

        # IRF.
        self._RstIRF = RstCloth()
        self._IRFFileName = "IRF"

        # IRF with speed.
        self._RstIRFspeed = RstCloth()
        self._IRFspeedFileName = "IRF_speed"

        # Path to the folder including hdbtool.
        my_path = os.path.abspath(os.path.dirname(__file__))

        # conf.py file for generating the html file.
        self._conf_file = my_path + "/conf.py" # In the frydom-CE deposit.
        conf_file = os.path.join(output_folder, 'conf.py')
        copyfile(self._conf_file, conf_file)

        # _static folder.
        self.static_folder = os.path.join(output_folder,'_static/')
        if not os.path.exists(self.static_folder):
            os.makedirs(self.static_folder)

        # Source folder.
        self.source_folder = os.path.join(output_folder, 'Source/')
        if not os.path.exists(self.source_folder):
            os.makedirs(self.source_folder)

        # Build folder.
        self.build_folder = os.path.join(output_folder, 'build/')

        # Pdf output file name.
        self.pdfname = os.path.splitext(latex_documents[0][1])[0]

        # Logo D-ice Engineering.
        Dice_logo = os.path.basename(latex_logo)
        copyfile(os.path.join(my_path, "../../../data/", Dice_logo) , os.path.join(self.static_folder, Dice_logo))

        # Wave direction convention.
        self.Wave_direction_file = "Wave_direction_convention.jpg"
        copyfile(os.path.join(my_path, "../../../docs/theory_manual/source/_static/", self.Wave_direction_file), os.path.join(self.static_folder, self.Wave_direction_file))

        # Wave direction convention.
        self.Notation_AB = "Notation_AB.png"
        copyfile(os.path.join(my_path, "../../../docs/theory_manual/source/_static/", self.Notation_AB), os.path.join(self.static_folder, self.Notation_AB))

    def WriteIndex(self, pyHDB):
        """This function writes the rst file Index.rst."""

        # Title.
        self._RstIndex.title("Hydrodynamic database results")
        self._RstIndex.newline()
        self._RstIndex.directive(name="toctree", fields=[('maxdepth', '3')])
        self._RstIndex.newline()
        self._RstIndex._add(".. Contents:")
        self._RstIndex.newline()

        # Convetions.
        self._RstIndex.directive(name="toctree", fields=[('maxdepth', '3')])
        self._RstIndex.newline()
        self._RstIndex.content('Source/' + self._ConventionsFileName, indent=3, block='ct2')
        self._RstIndex.newline()

        # Input parameters.
        # self._RstIndex.h1("Input parameters for the frequency-domain solver")
        # self._RstIndex.newline()
        self._RstIndex.directive(name="toctree", fields=[('maxdepth', '3')])
        self._RstIndex.newline()
        self._RstIndex.content('Source/'+self._InputParamFileName, indent=3, block='ct2')
        self._RstIndex.newline()

        # Added mass and damping results.
        # self._RstIndex.h1("Added mass and damping results")
        # self._RstIndex.newline()
        self._RstIndex.directive(name="toctree", fields=[('maxdepth', '3')])
        self._RstIndex.newline()
        self._RstIndex.content('Source/' + self._AddedMassFileName, indent=3, block='ct2')
        self._RstIndex.newline()

        # Diffraction load results.
        # self._RstIndex.h1("Diffraction load results")
        # self._RstIndex.newline()
        self._RstIndex.directive(name="toctree", fields=[('maxdepth', '3')])
        self._RstIndex.newline()
        self._RstIndex.content('Source/' + self._DiffractionFileName, indent=3, block='ct2')
        self._RstIndex.newline()

        # Froude-Krylov load results.
        # self._RstIndex.h1("Froude-Krylov load results")
        # self._RstIndex.newline()
        self._RstIndex.directive(name="toctree", fields=[('maxdepth', '3')])
        self._RstIndex.newline()
        self._RstIndex.content('Source/' + self._FroudeKrylovFileName, indent=3, block='ct2')
        self._RstIndex.newline()

        # Excitation load results.
        # self._RstIndex.h1("Excitation load results")
        # self._RstIndex.newline()
        self._RstIndex.directive(name="toctree", fields=[('maxdepth', '3')])
        self._RstIndex.newline()
        self._RstIndex.content('Source/' + self._ExcitationFileName, indent=3, block='ct2')
        self._RstIndex.newline()

        # Infinite added mass results.
        # self._RstIndex.h1("Infinite added mass results")
        # self._RstIndex.newline()
        self._RstIndex.directive(name="toctree", fields=[('maxdepth', '3')])
        self._RstIndex.newline()
        self._RstIndex.content('Source/' + self._InfAddedMassFileName, indent=3, block='ct2')
        self._RstIndex.newline()

        # Impulse response function results.
        # self._RstIndex.h1("Impulse response function results")
        # self._RstIndex.newline()
        self._RstIndex.directive(name="toctree", fields=[('maxdepth', '3')])
        self._RstIndex.newline()
        self._RstIndex.content('Source/' + self._IRFFileName, indent=3, block='ct2')
        self._RstIndex.newline()

        # # Impulse reponse function results with forward speed.
        # self._RstIndex.h1("Impulse response function results with forward speed")
        # self._RstIndex.newline()
        # self._RstIndex.directive(name="toctree", fields=[('maxdepth', '3')])
        # self._RstIndex.newline()
        # self._RstIndex.content('Source/' + self._IRFspeedFileName, indent=3, block='ct2')
        # self._RstIndex.newline()

        # Index and table.
        # self._RstIndex.h1("Indices and tables")
        # self._RstIndex.newline()
        # self._RstIndex.li([':ref:`genindex`'], bullet='*', block='li2')
        # self._RstIndex.li([':ref:`modindex`'], bullet='*', block='li2')
        # self._RstIndex.li([':ref:`search`'], bullet='*', block='li2')

    def WriteConventions(self, pyHDB, output_folder):
        """This function writes the conventions in Index.rst."""

        self._RstConventions.title("Conventions and Definitions")
        self._RstConventions.newline()
        self._RstConventions._add("This chapter presents the conventions and the definitions used for the hydrodynamic database computation.")
        self._RstConventions.newline()
        self._RstConventions.h2("Hydrodynamic solver")
        self._RstConventions.newline()
        self._RstConventions._add("The frequency-domain linear potential flow based solver used to obtained the hydrodynamic database base is **" + pyHDB.solver + "**.")
        if(pyHDB.solver == "Nemoh"):
            self._RstConventions._add("Additional information about the theory used in **Nemoh** may be found in [Babarit2015]_.")
            self._RstConventions._add("The source code of **Nemoh** can be downloaded here_.")
            self._RstConventions.newline()
            self._RstConventions._add(".. _here: https://lheea.ec-nantes.fr/logiciels-et-brevets/nemoh-presentation-192863.kjsp")
        elif(pyHDB.solver == "WAMIT"):
            self._RstConventions._add("Additional information about the theory used in **WAMIT** may be found in [WAMIT2016]_ and on the official website (link_).")
            self._RstConventions._add(".. _link: https://www.wamit.com")
        self._RstConventions.newline()
        self._RstConventions.h2("Conventions")
        self._RstConventions.newline()
        self._RstConventions._add(r"The wave propagation direction is defined in the next figure. :math:`\beta = 0` represents a wave coming from the positive x while :math:`\beta = 180` is for a wave coming from the negative x.")
        self._RstConventions.newline()
        self._RstConventions.directive(name="figure", arg="/_static/" + self.Wave_direction_file, fields=[('align', 'center'), ('scale', '50 %'), ('height', '800 px'), ('width', '750 px')])
        self._RstConventions.newline()
        self._RstConventions._add(r'   Sketch defining the global frame (N, E), the body coordinates (:math:`x`, :math:`y`) and the wave direction angle :math:`\beta`. '
                     'The wave propagation direction is showed by the curved arrow. The vertical axes are positive upward. '
                                  'The plane :math:`z = 0` represents the undisturbed free surface.')
        self._RstConventions.newline()
        self._RstConventions.h2("Definitions")
        self._RstConventions.newline()
        self._RstConventions._add("Each body has six degrees of freedom, represented by the symbols given in the following table.")
        self._RstConventions.newline()
        self._RstConventions.directive(name="list-table", arg="Symbol of the six degrees of freedom")
        self._RstConventions.newline()
        self._RstConventions._add("    * - **Degree of freedom**")
        self._RstConventions._add("      - **Symbol**")
        self._RstConventions._add("    * - Surge")
        self._RstConventions._add("      - :math:`x`")
        self._RstConventions._add("    * - Sway")
        self._RstConventions._add("      - :math:`y`")
        self._RstConventions._add("    * - Heave")
        self._RstConventions._add("      - :math:`z`")
        self._RstConventions._add("    * - Roll")
        self._RstConventions._add("      - :math:`\phi`")
        self._RstConventions._add("    * - Pitch")
        self._RstConventions._add(r"      - :math:`\theta`")
        self._RstConventions._add("    * - Yaw")
        self._RstConventions._add("      - :math:`\psi`")
        self._RstConventions.newline()
        # self._RstConventions._add("References:")
        # self._RstConventions.newline()
        if (pyHDB.solver == "Nemoh"):
            self._RstConventions._add(".. [Babarit2015] A. Babarit and G. Delhommeau. Theoretical and numerical aspects of the open source BEM solver NEMOH. In *Proceedings of the 11th European Wave and Tidal Energy Conference*, *EWTEC2015*, 2015.")
        elif(pyHDB.solver == "WAMIT"):
            self._RstConventions._add(".. [WAMIT2016] WAMIT User manual, Version 7.2., 2016.")
        self._RstConventions.newline()

        # Writing.
        self._RstConventions.write(os.path.join(self.source_folder, self._ConventionsFileName + self.Ext)) # Conventions.rst.

    def WriteInputParameters(self, pyHDB, output_folder):
        """This function writes the rst file Input_parameters.rst."""

        self._RstInputParam.title("Input parameters")
        self._RstInputParam.newline()
        self._RstInputParam._add("This chapter lists the input parameters used in the frequency-domain linear potential flow based solver **"+pyHDB.solver+"**."
                                 " The global parameters are listed in the next table.")
        self._RstInputParam.newline()
        self._RstInputParam.directive(name="list-table", arg="Global parameters used in **%s**" % (pyHDB.solver))
        self._RstInputParam.newline()
        self._RstInputParam._add("    * - **Parameter**")
        self._RstInputParam._add("      - **Value**")
        self._RstInputParam._add(r"    * - Water density (:math:`kg/m^3`)")
        self._RstInputParam._add(r"      - " + str(pyHDB.rho_water))
        self._RstInputParam._add(r"    * - Gravity constant (:math:`m/s^2`)")
        self._RstInputParam._add(r"      - " + str(pyHDB.grav))
        if (pyHDB.depth == float('inf')):
            self._RstInputParam._add(r"    * - Water depth (:math:`m`)")
            self._RstInputParam._add(r"      - Infinity")
        else:
            self._RstInputParam._add(r"    * - Water depth (:math:`m`)")
            self._RstInputParam._add(r"      - " + str(pyHDB.depth))
        self._RstInputParam._add(r"    * - Number of bodies")
        self._RstInputParam._add(r"      - " + str(pyHDB.nb_bodies))
        self._RstInputParam._add(r"    * - Number of wave frequencies")
        self._RstInputParam._add(r"      - " + str(pyHDB.nb_wave_freq))
        self._RstInputParam._add(r"    * - Minimum frequency (:math:`rad/s`)")
        self._RstInputParam._add(r"      - %.2f" % (pyHDB.min_wave_freq))
        self._RstInputParam._add(r"    * - Maximum frequency (:math:`rad/s`)")
        self._RstInputParam._add(r"      - %.2f" % (pyHDB.max_wave_freq))
        self._RstInputParam._add(r"    * - Number of wave directions")
        self._RstInputParam._add(r"      - " + str(pyHDB.nb_wave_dir))
        self._RstInputParam._add(r"    * - Minimum wave direction (deg)")
        self._RstInputParam._add(r"      - %.2f" % (pyHDB.min_wave_dir))
        self._RstInputParam._add(r"    * - Maximum wave direction (deg)")
        self._RstInputParam._add(r"      - %.2f" % (pyHDB.max_wave_dir))
        self._RstInputParam.newline()
        self._RstInputParam._add("The link between the body meshes and the body indexes are given in the following table.")
        self._RstInputParam.newline()
        self._RstInputParam.directive(name="list-table", arg="Correspondance between the body meshes and the body indexes")
        self._RstInputParam.newline()
        self._RstInputParam._add("    * - **Body mesh**")
        self._RstInputParam._add("      - **Index**")
        for body in pyHDB.bodies:
            meshname = os.path.split(body.mesh.name)[1]
            self._RstInputParam._add("    * - %s" % meshname)
            self._RstInputParam._add("      - %u" % (body.i_body + 1))
        self._RstInputParam.newline()

        if(pyHDB.nb_bodies > 1):
            self._RstInputParam._add("The specific parameters for each body are listed in the next sections.")
        else:
            self._RstInputParam._add("The specific parameters of the body are listed below.")
        self._RstInputParam.newline()

        # Loop over the bodies.
        for body in pyHDB.bodies:

            mesh_file = "Mesh_"+str(body.i_body)+".png"
            meshname = os.path.split(body.mesh.name)[1]
            self._RstInputParam.h1("Body " + str(body.i_body + 1))
            self._RstInputParam.newline()

            # Specific parameters.
            self._RstInputParam.directive(name="list-table", arg="Mesh and mask parameters of body %u" % (body.i_body + 1))
            self._RstInputParam.newline()
            self._RstInputParam._add("    * - **Parameter**")
            self._RstInputParam._add("      - **value**")
            self._RstInputParam._add("    * - Mesh name")
            self._RstInputParam._add("      - " + str(meshname))
            self._RstInputParam._add("    * - Number of vertices")
            self._RstInputParam._add("      - " +  str(body.mesh.nb_vertices))
            self._RstInputParam._add("    * - Number of faces")
            self._RstInputParam._add("      - " + str(body.mesh.nb_faces))
            self._RstInputParam._add("    * - Motion mask")
            self._RstInputParam._add("      - " + str(body.Motion_mask))
            self._RstInputParam._add("    * - Force mask")
            self._RstInputParam._add(r"      - " + str(body.Force_mask))
            self._RstInputParam.newline()

            # Hydrostatic stiffness matrix.
            if(body._hydrostatic is not None):
                self._RstInputParam._add("The hydrostatic stiffness matrix is:")
                self._RstInputParam.newline()
                self._RstInputParam.directive('math', block='d0')
                self._RstInputParam.newline()
                self._RstInputParam._add(r"    K_{%u} = \begin{bmatrix}" % (body.i_body+1))
                self._RstInputParam._add(r"                0 & 0 & 0 & 0 & 0 & 0 \\")
                self._RstInputParam._add(r"                0 & 0 & 0 & 0 & 0 & 0 \\")
                self._RstInputParam._add(r"                0 & 0 & %.2f & %.2f & %.2f & 0 \\" % (body.hydrostatic.matrix33[0, 0], body.hydrostatic.matrix33[0, 1], body.hydrostatic.matrix33[0, 2]))
                self._RstInputParam._add(r"                0 & 0 & %.2f & %.2f & %.2f & 0 \\" % (body.hydrostatic.matrix33[1, 0], body.hydrostatic.matrix33[1, 1], body.hydrostatic.matrix33[1, 2]))
                self._RstInputParam._add(r"                0 & 0 & %.2f & %.2f & %.2f & 0 \\" % (body.hydrostatic.matrix33[2, 0], body.hydrostatic.matrix33[2, 1], body.hydrostatic.matrix33[2, 2]))
                self._RstInputParam._add(r"                0 & 0 & 0 & 0 & 0 & 0 \\")
                self._RstInputParam._add(r"             \end{bmatrix}")
                self._RstInputParam.newline()

            # Inertia matrix.
            if (body._inertia is not None):
                self._RstInputParam._add("The mass matrix is:")
                self._RstInputParam.newline()
                self._RstInputParam.directive('math', block='d0')
                self._RstInputParam.newline()
                self._RstInputParam._add(r"    M_{%u} = \begin{bmatrix}" % (body.i_body+1))
                self._RstInputParam._add(r"                %.2f & 0 & 0 & 0 & 0 & 0 \\" % body.inertia.mass)
                self._RstInputParam._add(r"                0 & %.2f & 0 & 0 & 0 & 0 \\" % body.inertia.mass)
                self._RstInputParam._add(r"                0 & 0 & %.2f & 0 & 0 & 0 \\" % body.inertia.mass)
                self._RstInputParam._add(r"                0 & 0 & 0 & %.2f & %.2f & %.2f \\" % (body.inertia.matrix33[0, 0], body.inertia.matrix33[0, 1], body.inertia.matrix33[0, 2]))
                self._RstInputParam._add(r"                0 & 0 & 0 & %.2f & %.2f & %.2f \\" % (body.inertia.matrix33[1, 0], body.inertia.matrix33[1, 1], body.inertia.matrix33[1, 2]))
                self._RstInputParam._add(r"                0 & 0 & 0 & %.2f & %.2f & %.2f \\" % (body.inertia.matrix33[2, 0], body.inertia.matrix33[2, 1], body.inertia.matrix33[2, 2]))
                self._RstInputParam._add(r"             \end{bmatrix}")
                self._RstInputParam.newline()

            # Screenshot of the mesh.
            self._RstInputParam.directive(name="figure", arg="/_static/" + mesh_file, fields=[('align', 'center')])
            self._RstInputParam.newline()

            # Caption.
            self._RstInputParam._add('   Mesh of body ' + str(body.i_body + 1))
            self._RstInputParam.newline()

            # Picture of the mesh.
            self.PlotMesh(body, output_folder, mesh_file)

        # Writing.
        self._RstInputParam.write(os.path.join(self.source_folder, self._InputParamFileName + self.Ext))  # Input_parameters.rst.

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
        print("")

        # Copy of the mesh in the _Static folder.
        copyfile("screenshot.png", self.static_folder + mesh_file) # "screenshot.png" is hard codded in the script MMviewer.py of Meshmagick.

        # Deletion of the screenshot.
        cwd = os.getcwd()
        os.remove(cwd + "/screenshot.png")

    def WriteHDB(self, pyHDB, output_folder):
        """This function writes the *.rst files for presenting the hydrodynamic database."""

        # Added mass and Damping.
        self.WriteAddedMassDamping(pyHDB, output_folder)
        self._RstAddedMass.write(os.path.join(self.source_folder, self._AddedMassFileName + self.Ext)) # Added_mass_Damping.rst.

        # Diffraction loads.
        self.WriteLoads(pyHDB, output_folder, self._RstDiffraction, 0)
        self._RstDiffraction.write(os.path.join(self.source_folder, self._DiffractionFileName + self.Ext)) # Diffraction.rst.

        # Froude-Krylov loads.
        self.WriteLoads(pyHDB, output_folder, self._RstFroudeKrylov, 1)
        self._RstFroudeKrylov.write(os.path.join(self.source_folder, self._FroudeKrylovFileName + self.Ext)) # Froude_Krylov.rst.

        # Excitation loads.
        self.WriteLoads(pyHDB, output_folder, self._RstExcitation, 2)
        self._RstExcitation.write(os.path.join(self.source_folder, self._ExcitationFileName + self.Ext)) # Excitation.rst.

        # Infinite added mass matrix.
        self.WriteInfAddedMass(pyHDB, output_folder, self._RstInfAddedMass)
        self._RstInfAddedMass.write(os.path.join(self.source_folder, self._InfAddedMassFileName + self.Ext)) # Inf_Added_mass.rst.

        # IRF.
        self.WriteIRF(pyHDB, output_folder, self._RstIRF, 0)
        self._RstIRF.write(os.path.join(self.source_folder, self._IRFFileName + self.Ext)) # IRF.rst.

        # # IRF with forward speed.
        # self.WriteIRF(pyHDB, output_folder, self._RstIRFspeed, 1)
        # self._RstIRFspeed.write(os.path.join(self.source_folder, self._IRFspeedFileName + self.Ext)) # IRF_speed.rst.

    def WriteAddedMassDamping(self, pyHDB, output_folder):
        """ This function writes the added-mass and damping results in Added_mass_Damping.rst."""

        self._RstAddedMass.title("Added mass and damping")
        self._RstAddedMass.newline()
        self._RstAddedMass._add("This chapter presents the added mass and damping results. The following notation is adopted:")
        self._RstAddedMass.newline()
        self._RstAddedMass.directive(name="figure", arg="/_static/" + self.Notation_AB, fields=[('align', 'center'), ('height', '40 px'), ('width', '70 px')])
        self._RstAddedMass.newline()
        self._RstAddedMass._add(r"which representes the added mass (:math:`X = A`) or damping (:math:`X = B`) coefficient for a a unit acceleration of body :math:`j` "
                                r"along the degree of freedom :math:`\beta` that radiates waves and generates a force on body :math:`i` along the degree of freedom :math:`\alpha`.")
        self._RstAddedMass.newline()

        for ibody_force in range(0, pyHDB.nb_bodies):

            self._RstAddedMass.h1("Body " + str(ibody_force + 1))
            self._RstAddedMass.newline()

            for iforce in range(0, 6):
                for ibody_motion in range(0, pyHDB.nb_bodies):

                    ABfile = "AB_"+str(ibody_force)+str(iforce)+str(ibody_motion)+".png"

                    # Data.
                    data = np.zeros((pyHDB.nb_wave_freq + 1, 12), dtype=np.float)  # 12 because 6 the for added mass and 6 for the damping coefficients, +1 for the infinite added mass.
                    for idof in range(0,6):
                        data[0:pyHDB.nb_wave_freq, idof] = pyHDB.bodies[ibody_motion].Added_mass[iforce, 6 * ibody_motion + idof, :]
                        data[pyHDB.nb_wave_freq, idof] = pyHDB.bodies[ibody_motion].Inf_Added_mass[iforce, 6 * ibody_motion + idof]
                        data[0:pyHDB.nb_wave_freq, 6 + idof] = pyHDB.bodies[ibody_motion].Damping[iforce, 6 * ibody_motion + idof, :]

                    # Plots.
                    plot_AB_multiple_coef(data, pyHDB.wave_freq, ibody_force, iforce, ibody_motion, show = False, save = True, filename = self.static_folder+ABfile)

                    self._RstAddedMass.directive(name="figure", arg="/_static/" + ABfile, fields=[('align', 'center')])
                    self._RstAddedMass.newline()

                    if (iforce <= 2):
                        force_str = 'force'
                    else:
                        force_str = 'moment'

                    # Caption.
                    self._RstAddedMass._add('   Added mass (top) and damping (bottom) coefficients giving ' + force_str + ' in ' + Dof_name[iforce]
                                             + ' for body ' + str(ibody_force + 1)
                                             + " generated by a motion of body " + str(ibody_motion + 1)
                                             + ". The crosses represent the infinite added mass coefficients.")
                    self._RstAddedMass.newline()

    def WriteLoads(self, pyHDB, output_folder, RSTfile, DiffOrFKOrExc):
        """ This function writes the diffraction or Froude-Krylov or excitation results in a *.rst file.

        Parameters
        ----------
        RSTfile : RST object.
            RST object to write the loads.
        DiffOrFKOrExc : int.
            0 for diffraction loads, 1 for Froude-Krylov loads, 2 for excitation loads.
        """

        if(DiffOrFKOrExc == 0): # Diffraction.
            RSTfile.title("Diffraction loads")
            RSTfile.newline()
            RSTfile._add("This chapter presents the diffraction load results.")
            FilenameMaj = "Diffraction"
            FilenameMin = "diffraction"
        elif (DiffOrFKOrExc == 1): # Froude-Krylov.
            RSTfile.title("Froude-Krylov loads")
            RSTfile.newline()
            RSTfile._add("This chapter presents the Froude-Krylov load results.")
            FilenameMaj = "Froude-Krylov"
            FilenameMin = "Froude-Krylov"
        else: # Excitation.
            RSTfile.title("Excitation loads")
            RSTfile.newline()
            RSTfile._add("This chapter presents the excitation load results.")
            FilenameMaj = "Excitation"
            FilenameMin = "excitation"
        RSTfile.newline()

        for body in pyHDB.bodies:

            RSTfile.h1("Body " + str(body.i_body + 1))
            RSTfile.newline()

            # Loads.
            if (DiffOrFKOrExc == 0): # Diffraction.
                Loads = body.Diffraction
            elif (DiffOrFKOrExc == 1): # Froude-Krylov.
                Loads = body.Froude_Krylov
            else: # Excitation.
                Loads = Loads = body.Diffraction + body.Froude_Krylov

            for iforce in range(0, 6):

                Loadsfile = FilenameMaj+"_" + str(body.i_body) + str(iforce) + ".png"

                # Data by interpolation about the wave directions.
                f_interp_Loads = interpolate.interp1d(pyHDB.wave_dir, Loads[iforce, :, :], axis=1)  # axis = 1 -> wave directions.
                data = f_interp_Loads(np.radians(Beta_report))

                # Plot.
                plot_loads_all_wave_dir(data, pyHDB.wave_freq, DiffOrFKOrExc, body.i_body, iforce, Beta_report, show = False, save = True, filename = self.static_folder+Loadsfile)

                RSTfile.directive(name="figure", arg="/_static/" + Loadsfile, fields=[('align', 'center')])
                RSTfile.newline()

                if (iforce <= 2):
                    force_str = 'force'
                else:
                    force_str = 'moment'

                # Caption.
                RSTfile._add('   Amplitude (top) and phase (bottom) of the ' + FilenameMin + ' ' + force_str + " in " + Dof_name[iforce] + ' of body ' + str(body.i_body + 1))
                RSTfile.newline()

    def WriteInfAddedMass(self, pyHDB, output_folder, RSTfile):
        """ This function writes the infinite added mass matrix in a *.rst file.

        Parameter
        ---------
        RSTfile : RST object.
            RST object to write the loads.
        """

        RSTfile.title("Infinite added mass matrices")
        RSTfile.newline()
        RSTfile._add("This chapter presents the infinite added mass matrix for each body.")
        RSTfile.newline()

        # Loop over the bodies.
        for body_force in pyHDB.bodies:
            if (body_force.Inf_Added_mass is not None):
                RSTfile.h1("Body " + str(body_force.i_body + 1))
                RSTfile.newline()

                for body_motion in pyHDB.bodies:

                    ibody_motion = body_motion.i_body

                    RSTfile.directive('math', block='d0')
                    RSTfile.newline()
                    RSTfile._add(r"    A_{%s}^{\infty} = \begin{bmatrix}" % (str(body_force.i_body+1) + str(ibody_motion+1)))
                    RSTfile._add(r"                %.2f & %.2f & %.2f & %.2f & %.2f & %.2f \\" % (body_force.Inf_Added_mass[0, 6 * ibody_motion + 0], body_force.Inf_Added_mass[0, 6 * ibody_motion + 1],
                                 body_force.Inf_Added_mass[0, 6 * ibody_motion + 2], body_force.Inf_Added_mass[0, 6 * ibody_motion + 3],
                                 body_force.Inf_Added_mass[0, 6 * ibody_motion + 4], body_force.Inf_Added_mass[0, 6 * ibody_motion + 5]))
                    RSTfile._add(r"                %.2f & %.2f & %.2f & %.2f & %.2f & %.2f \\" % (body_force.Inf_Added_mass[1, 6 * ibody_motion + 0], body_force.Inf_Added_mass[1, 6 * ibody_motion + 1],
                                 body_force.Inf_Added_mass[1, 6 * ibody_motion + 2], body_force.Inf_Added_mass[1, 6 * ibody_motion + 3],
                                 body_force.Inf_Added_mass[1, 6 * ibody_motion + 4], body_force.Inf_Added_mass[1, 6 * ibody_motion + 5]))
                    RSTfile._add(r"                %.2f & %.2f & %.2f & %.2f & %.2f & %.2f \\" % (body_force.Inf_Added_mass[2, 6 * ibody_motion + 0], body_force.Inf_Added_mass[2, 6 * ibody_motion + 1],
                                 body_force.Inf_Added_mass[2, 6 * ibody_motion + 2], body_force.Inf_Added_mass[2, 6 * ibody_motion + 3],
                                 body_force.Inf_Added_mass[2, 6 * ibody_motion + 4], body_force.Inf_Added_mass[2, 6 * ibody_motion + 5]))
                    RSTfile._add(r"                %.2f & %.2f & %.2f & %.2f & %.2f & %.2f \\" % (body_force.Inf_Added_mass[3, 6 * ibody_motion + 0], body_force.Inf_Added_mass[3, 6 * ibody_motion + 1],
                                 body_force.Inf_Added_mass[3, 6 * ibody_motion + 2], body_force.Inf_Added_mass[3, 6 * ibody_motion + 3],
                                 body_force.Inf_Added_mass[3, 6 * ibody_motion + 4], body_force.Inf_Added_mass[3, 6 * ibody_motion + 5]))
                    RSTfile._add(r"                %.2f & %.2f & %.2f & %.2f & %.2f & %.2f \\" % (body_force.Inf_Added_mass[4, 6 * ibody_motion + 0], body_force.Inf_Added_mass[4, 6 * ibody_motion + 1],
                                 body_force.Inf_Added_mass[4, 6 * ibody_motion + 2], body_force.Inf_Added_mass[4, 6 * ibody_motion + 3],
                                 body_force.Inf_Added_mass[4, 6 * ibody_motion + 4], body_force.Inf_Added_mass[4, 6 * ibody_motion + 5]))
                    RSTfile._add(r"                %.2f & %.2f & %.2f & %.2f & %.2f & %.2f \\" % (body_force.Inf_Added_mass[5, 6 * ibody_motion + 0], body_force.Inf_Added_mass[5, 6 * ibody_motion + 1],
                                 body_force.Inf_Added_mass[5, 6 * ibody_motion + 2], body_force.Inf_Added_mass[5, 6 * ibody_motion + 3],
                                 body_force.Inf_Added_mass[5, 6 * ibody_motion + 4], body_force.Inf_Added_mass[5, 6 * ibody_motion + 5]))
                    RSTfile._add(r"             \end{bmatrix}")
                    RSTfile.newline()

    def WriteIRF(self, pyHDB, output_folder, RSTfile, SpeedOrNot):
        """ This function writes the diffraction or Froude-Krylov or excitation results in a *.rst file.

        Parameters
        ----------
        RSTfile : RST object.
            RST object to write the loads.
        SpeedOrNot : int
            IRF with forward speed (1) or not (0).
        """

        if (SpeedOrNot == 0): # IRF without forward speed.
            RSTfile.title("Impulse response functions")
            RSTfile.newline()
            RSTfile._add("This chapter presents the impulse response function results.")
            Filename = "IRF"
        else: # IRF with forward speed.
            RSTfile.title("Impulse response function with forward speed")
            RSTfile.newline()
            RSTfile._add("This chapter presents the impulse response function results with forward speed.")
            Filename = "IRF_speed"
        RSTfile.newline()

        for ibody_force in range(0, pyHDB.nb_bodies):

            RSTfile.h1("Body " + str(ibody_force + 1))
            RSTfile.newline()

            for iforce in range(0, 6):
                for ibody_motion in range(0, pyHDB.nb_bodies):

                        IRFfile = Filename+"_"+str(ibody_force)+str(iforce)+str(ibody_motion)+".png"

                        # Data.
                        data = np.zeros((pyHDB.nb_time_samples, 6), dtype=np.float)
                        for idof in range(0, 6):
                            if(SpeedOrNot == 0): # IRF without forward speed.
                                data[:, idof] = pyHDB.bodies[ibody_force].irf[iforce, 6 * ibody_motion + idof, :]
                            else: # IRF with forward speed.
                                data[:, idof] = pyHDB.bodies[ibody_force].irf_ku[iforce, 6 * ibody_motion + idof, :]

                        # Plots.
                        plot_irf_multiple_coef(data, pyHDB.time, SpeedOrNot, ibody_force, iforce, ibody_motion, show = False, save = True, filename = self.static_folder+IRFfile)

                        RSTfile.directive(name="figure", arg="/_static/" + IRFfile, fields=[('align', 'center')])
                        RSTfile.newline()

                        if (iforce <= 2):
                            force_str = 'force'
                        else:
                            force_str = 'moment'

                        # Caption.
                        if(SpeedOrNot == 0): # IRF without forward speed.
                            RSTfile._add('   Impulse response function of ' + force_str + ' in ' + Dof_name[iforce] + ' of body ' + str(ibody_force + 1)
                                                     + " for a motion " + " of body " + str(ibody_motion + 1))
                        else: # IRF with forward speed.
                            RSTfile._add('   Impulse response function with forward speed of ' + force_str + ' in ' + Dof_name[iforce] + 'of body ' + str(ibody_force + 1)
                                         + " for a motion of body " + str(ibody_motion + 1))

                        RSTfile.newline()

    def WriteIndexRst(self, output_folder):
        """This functions writes the index.rst file."""

        self._RstIndex.write(os.path.join(output_folder, self._IndexFileName + self.Ext)) # index.rst.

    def BuildHTML(self, output_folder):
        """This function builds the html file from the rst files."""

        os.system('sphinx-build -b html '+ str(output_folder) + ' ' + self.build_folder)

    def OpenHTML(self, output_folder):
        """This function opens the html file in a web browser."""

        html_file = os.path.join(output_folder,'build/',self._IndexFileName+'.html')
        webbrowser.open(html_file)

    def BuildPDF(self, output_folder):
        """This function writes a latex latex and then builds the pdf output file from the latex file."""

        build_folder = os.path.join(output_folder, 'build')
        os.system('sphinx-build -M latexpdf ' + str(output_folder) + ' ' + self.build_folder)
        pdf_file = os.path.join(build_folder, 'latex', self.pdfname + '.pdf')
        copyfile(pdf_file, os.path.join(build_folder, self.pdfname + '.pdf'))
