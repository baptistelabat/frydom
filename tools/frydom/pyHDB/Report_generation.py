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

import meshmagick.MMviewer
from plot_db import *

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

        # Input parameters file.
        self._RstInputParam = RstCloth()
        self._InputParamFileName = "Input_parameters"

        # HDB results.
        self._RstHDB = RstCloth()
        self._HDBFileName = "HDB_results"

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

        # Post-processing results.
        self._RstPP = RstCloth()
        self._PPFileName = "PP_results"

        # IRF.
        self._RstIRF = RstCloth()
        self._IRFFileName = "IRF"

        # IRF with speed.
        self._RstIRFspeed = RstCloth()
        self._IRFspeedFileName = "IRF_speed"

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
        now = datetime.datetime.now()
        self._RstIndex._add("Report generated date: " + str(now.strftime("%Y-%m-%d")))
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
        self._RstIndex.h1("Hydrodynamic database results")
        self._RstIndex.newline()
        self._RstIndex.directive(name="toctree", fields=[('maxdepth', '3')])
        self._RstIndex.newline()
        self._RstIndex.content('Source/' + self._HDBFileName, indent=3, block='ct2')
        self._RstIndex.newline()
        self._RstIndex.h1("Post-processing results")
        self._RstIndex.newline()
        self._RstIndex.directive(name="toctree", fields=[('maxdepth', '3')])
        self._RstIndex.newline()
        self._RstIndex.content('Source/' + self._PPFileName, indent=3, block='ct2')
        self._RstIndex.newline()
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
            self._RstInputParam._add('Water depth                ' + str(pyHDB.depth) + ' m')
        self._RstInputParam._add('Number of bodies           ' + str(pyHDB.nb_bodies))
        self._RstInputParam._add('Number of wave frequencies ' + str(pyHDB.nb_wave_freq))
        self._RstInputParam._add('Number of wave directions  ' + str(pyHDB.nb_wave_dir))
        self._RstInputParam._add('========================== ==================================')
        self._RstInputParam.newline()

        if(pyHDB.nb_bodies > 1):
            self._RstInputParam._add("The main parameters of the bodies are listed below.")
        else:
            self._RstInputParam._add("The main parameters of the body are listed below.")
        self._RstInputParam.newline()

        # Loop over the bodies.
        for body in pyHDB.bodies:

            mesh_file = "Mesh_"+str(body.i_body)+".png"
            self._RstInputParam.h1("Body " + str(body.i_body + 1))
            self._RstInputParam.newline()
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
            self._RstInputParam.directive(name="figure", arg = "/_static/" + mesh_file,  fields=[('align', 'center')])
            self._RstInputParam.newline()

            # Caption.
            self._RstInputParam._add('   Mesh of body ' + str(body.i_body + 1))
            self._RstInputParam.newline()

            # Picture of the mesh.
            self.PlotMesh(body, output_folder, mesh_file)

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
        """This function writes the hdb result in HDB_result.rst."""

        self._RstHDB.title("HDB results")
        self._RstHDB.newline()
        self._RstHDB._add("This chapter presents the results of the hydrodynamic database.")
        self._RstHDB.newline()
        self._RstHDB.directive(name="toctree", fields=[('maxdepth', '3')])
        self._RstHDB.newline()

        # Added mass and Damping.
        self._RstHDB.content('../Source/' + self._AddedMassFileName, indent=3, block='ct2')
        self._RstHDB.newline()
        self.WriteAddedMassDamping(pyHDB, output_folder)

        # Diffraction loads.
        self._RstHDB.content('../Source/' + self._DiffractionFileName, indent=3, block='ct2')
        self._RstHDB.newline()
        self.WriteLoads(pyHDB, output_folder, self._RstDiffraction, 0)

        # Froude-Krylov loads.
        self._RstHDB.content('../Source/' + self._FroudeKrylovFileName, indent=3, block='ct2')
        self._RstHDB.newline()
        self.WriteLoads(pyHDB, output_folder, self._RstFroudeKrylov, 1)

        # Excitation loads.
        self._RstHDB.content('../Source/' + self._ExcitationFileName, indent=3, block='ct2')
        self._RstHDB.newline()
        self.WriteLoads(pyHDB, output_folder, self._RstExcitation, 2)

        # Post-processing results.
        self._RstPP.title("Post-processing results")
        self._RstPP.newline()
        self._RstPP._add("This chapter presents the post-processing results obtained from the hydrodynamic database.")
        self._RstPP.newline()
        self._RstPP.directive(name="toctree", fields=[('maxdepth', '3')])
        self._RstPP.newline()

        # IRF.
        self._RstPP.content('../Source/' + self._IRFFileName, indent=3, block='ct2')
        self._RstPP.newline()
        self.WriteIRF(pyHDB, output_folder, self._RstIRF, 0)

        # IRF with speed.
        self._RstPP.content('../Source/' + self._IRFspeedFileName, indent=3, block='ct2')
        self._RstPP.newline()
        self.WriteIRF(pyHDB, output_folder, self._RstIRFspeed, 1)

    def WriteAddedMassDamping(self, pyHDB, output_folder):
        """ This function writes the added-mass and damping results in Added_mass_Damping.rst."""

        self._RstAddedMass.title("Added mass and damping")
        self._RstAddedMass.newline()
        self._RstAddedMass._add("This section presents the added mass and damping results.")
        self._RstAddedMass.newline()

        for ibody_force in range(0, pyHDB.nb_bodies):

            self._RstAddedMass.h1("Body " + str(ibody_force + 1))
            self._RstAddedMass.newline()

            for iforce in range(0, 6):
                for ibody_motion in range(0, pyHDB.nb_bodies):
                    for idof in range(0, 6):

                        ABfile = "AB_"+str(ibody_force)+str(iforce)+str(ibody_motion)+str(idof)+".png"

                        # Data.
                        data = np.zeros((pyHDB.nb_wave_freq + 1, 2), dtype=np.float)  # 2 for added mass and damping coefficients, +1 for the infinite added mass.
                        data[0:pyHDB.nb_wave_freq, 0] = pyHDB.bodies[ibody_motion].Added_mass[iforce, 6 * ibody_force + idof, :]
                        data[pyHDB.nb_wave_freq, 0] = pyHDB.bodies[ibody_motion].Inf_Added_mass[iforce, 6 * ibody_force + idof]
                        data[0:pyHDB.nb_wave_freq, 1] = pyHDB.bodies[ibody_motion].Damping[iforce, 6 * ibody_force + idof, :]

                        # Plots.
                        plot_AB(data, pyHDB.wave_freq, ibody_force, iforce, ibody_motion, idof, show = False, save = True, filename = self.static_folder+ABfile)

                        self._RstAddedMass.directive(name="figure", arg="/_static/" + ABfile, fields=[('align', 'center')])
                        self._RstAddedMass.newline()

                        if (iforce <= 2):
                            force_str = 'force'
                            if (idof <= 2): # Translation.
                                motion_str = 'translation'
                            else: # Rotation.
                                motion_str = 'rotation'
                        else:
                            force_str = 'moment'
                            if (idof <= 2): # Translation.
                                motion_str = 'translation'
                            else: # Rotation.
                                motion_str = 'rotation'

                        # Caption.
                        self._RstAddedMass._add('   Added mass (top) and damping (bottom) coefficients giving ' + force_str+' on body ' + str(ibody_force + 1)
                                                 + " along direction " + str(iforce + 1) + " for a " + motion_str+" of body " + str(ibody_motion + 1)
                                                 + " along direction " + str(idof + 1) + ". The red cross represents the infinite added mass coefficient.")
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
            RSTfile._add("This section presents the diffraction results.")
            FilenameMaj = "Diffraction"
            FilenameMin = "diffraction"
        elif (DiffOrFKOrExc == 1): # Froude-Krylov.
            RSTfile.title("Froude-Krylov loads")
            RSTfile.newline()
            RSTfile._add("This section presents the diffraction results.")
            FilenameMaj = "Froude-Krylov"
            FilenameMin = "Froude-Krylov"
        else: # Excitation.
            RSTfile.title("Excitation loads")
            RSTfile.newline()
            RSTfile._add("This section presents the excitation results.")
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

            for iwave in range(0, pyHDB.nb_wave_dir):

                for iforce in range(0, 6):

                    Loadsfile = FilenameMaj+"_" + str(body.i_body) + str(iwave) + str(iforce) + ".png"

                    # Data.
                    data = Loads[iforce, :, iwave]

                    # Wave direction.
                    beta = np.degrees(pyHDB.wave_dir[iwave])

                    # Plot.
                    plot_loads(data, pyHDB.wave_freq, DiffOrFKOrExc, body.i_body, iforce, beta, show = False, save = True, filename = self.static_folder+Loadsfile)

                    RSTfile.directive(name="figure", arg="/_static/" + Loadsfile, fields=[('align', 'center')])
                    RSTfile.newline()

                    if (iforce <= 2):
                        force_str = 'force'
                    else:
                        force_str = 'moment'

                    # Caption.
                    RSTfile._add('   Amplitude (top) and phase (bottom) of the ' + FilenameMin + ' ' + force_str + ' on body ' + str(body.i_body + 1)
                                            + " along direction " + str(iforce + 1) + " for a wave direction of %.1f deg" % beta)
                    RSTfile.newline()

    def WriteIRF(self, pyHDB, output_folder, RSTfile, SpeedOrNot):
        """ This function writes the diffraction or Froude-Krylov or excitation results in a *.rst file.

        Parameters
        ----------
        RSTfile : RST object.
            RST object to write the loads.
        peedOrNot : int
            IRF with forward speed (1) or not (0).
        """

        if (SpeedOrNot == 0): # IRF without forward speed.
            RSTfile.title("Impulse response functions")
            RSTfile.newline()
            RSTfile._add("This section presents the impulse response function results.")
            Filename = "IRF"
        else: # IRF with forward speed.
            RSTfile.title("Impulse response function with forward speed")
            RSTfile.newline()
            RSTfile._add("This section presents the impulse response function results with forward speed.")
            Filename = "IRF_speed"
        RSTfile.newline()

        for ibody_force in range(0, pyHDB.nb_bodies):

            RSTfile.h1("Body " + str(ibody_force + 1))
            RSTfile.newline()

            for iforce in range(0, 6):
                for ibody_motion in range(0, pyHDB.nb_bodies):
                    for idof in range(0, 6):

                        IRFfile = Filename+"_"+str(ibody_force)+str(iforce)+str(ibody_motion)+str(idof)+".png"

                        # Data.
                        if(SpeedOrNot == 0): # IRF without forward speed.
                            data = pyHDB.bodies[ibody_force].irf[iforce, 6 * ibody_motion + idof, :]
                        else: # IRF with forward speed.
                            data = pyHDB.bodies[ibody_force].irf_ku[iforce, 6 * ibody_motion + idof, :]

                        # Plots.
                        plot_irf(data, pyHDB.time, SpeedOrNot, ibody_force, iforce, ibody_motion, idof, show = False, save = True, filename = self.static_folder+IRFfile)

                        RSTfile.directive(name="figure", arg="/_static/" + IRFfile, fields=[('align', 'center')])
                        RSTfile.newline()

                        if (iforce <= 2):
                            force_str = 'force'
                            if (idof <= 2):  # Translation.
                                motion_str = 'translation'
                            else:  # Rotation.
                                motion_str = 'rotation'
                        else:
                            force_str = 'moment'
                            if (idof <= 2):  # Translation.
                                motion_str = 'translation'
                            else:  # Rotation.
                                motion_str = 'rotation'

                        # Caption.
                        if(SpeedOrNot == 0): # IRF without forward speed.
                            RSTfile._add('   Impulse response function of body ' + str(ibody_force + 1)
                                                     + " along direction " + str(iforce + 1) + " for a " + motion_str+" of body " + str(ibody_motion + 1)
                                                     + " along direction " + str(idof + 1))
                        else: # IRF with forward speed.
                            RSTfile._add('   Impulse response function with forward speed of body ' + str(ibody_force + 1)
                                         + " along direction " + str(iforce + 1) + " for a " + motion_str + " of body " + str(ibody_motion + 1)
                                         + " along direction " + str(idof + 1))

                        RSTfile.newline()

    def WriteRst(self, output_folder):
        """This functions writes the rst files except HDB_result.rst because of the RAO and the drift loads."""

        self._RstIndex.write(os.path.join(output_folder, self._IndexFileName + self.Ext)) # Index.rst.
        self._RstInputParam.write(os.path.join(self.source_folder, self._InputParamFileName + self.Ext)) # Input_parameters.rst.
        self._RstHDB.write(os.path.join(self.source_folder, self._HDBFileName + self.Ext))  # HDB_results.rst.
        self._RstAddedMass.write(os.path.join(self.source_folder, self._AddedMassFileName + self.Ext)) # Added_mass_Damping.rst.
        self._RstDiffraction.write(os.path.join(self.source_folder, self._DiffractionFileName + self.Ext)) # Diffraction.rst.
        self._RstFroudeKrylov.write(os.path.join(self.source_folder, self._FroudeKrylovFileName + self.Ext)) # Froude_Krylov.rst.
        self._RstExcitation.write(os.path.join(self.source_folder, self._ExcitationFileName + self.Ext)) # Excitation.rst.
        self._RstIRF.write(os.path.join(self.source_folder, self._IRFFileName + self.Ext)) # IRF.rst.
        self._RstIRFspeed.write(os.path.join(self.source_folder, self._IRFspeedFileName + self.Ext)) # IRF_speed.rst.

    def WritePPRst(self, output_folder):
        """This functions writes HDB_results.rst."""

        self._RstPP.write(os.path.join(self.source_folder, self._PPFileName + self.Ext)) # PP_results.rst.

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