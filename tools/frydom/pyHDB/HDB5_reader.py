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
    Module to load a HDB5 file.
"""

import numpy as np
import h5py

from meshmagick.mesh import Mesh

from body_db_v2 import *
from wave_drift_db_v2 import WaveDriftDB

class HDB5reader():
    """
        Class for reading HDB5 file..
    """

    def __init__(self, pyHDB, hdb5_file):
        """ Constructor of the class NemohReader.

         Parameters
        ----------
        pyHDB : object
            pyHDB object for storing the hydrodynamic database.
        hdb5_file : string
            Path to the hdb5 file to load.
        """

        with h5py.File(hdb5_file, 'r') as reader:

            # Version.
            self.read_version(reader, pyHDB)

            # Environment.
            self.read_environment(reader, pyHDB)

            # Discretization.
            self.read_discretization(reader, pyHDB)

            # Bodies.
            self.read_bodies(reader, pyHDB)

            # Wave drift coefficients.
            self.read_wave_drift(reader, pyHDB, "/WaveDrift")

    def read_environment(self, reader, pyHDB):
        """This function reads the environmental data of the *.hdb5 file.

        Parameter
        ---------
        reader : string.
            *.hdb5 file.
        pyHDB : object
            pyHDB object for storing the hydrodynamic database.
        """

        # Date.
        self.Creation_data_hdf5file = np.array(reader['CreationDate']) # Date of creation of the hdf5file.

        # Gravity acceleration.
        pyHDB.grav = np.array(reader['GravityAcc'])

        # Water density.
        pyHDB.rho_water = np.array(reader['WaterDensity'])

        # Normalisation length.
        pyHDB.normalization_length = np.array(reader['NormalizationLength'])

        # Water depth.
        pyHDB.depth = np.array(reader['WaterDepth'])

        # Number of bodies.
        pyHDB.nb_bodies = np.array(reader['NbBody'])

    def read_discretization(self, reader, pyHDB):
        """This function reads the discretization parameters of the *.hdb5 file.

        Parameter
        ---------
        reader : string.
            *.hdb5 file.
        pyHDB : object
            pyHDB object for storing the hydrodynamic database.
        """

        discretization_path = "/Discretizations"

        # Frequency discretization.

        frequential_path = discretization_path + "/Frequency"

        pyHDB.nb_wave_freq = np.array(reader[frequential_path + "/NbFrequencies"])
        pyHDB.min_wave_freq = np.array(reader[frequential_path + "/MinFrequency"])
        pyHDB.max_wave_freq = np.array(reader[frequential_path + "/MaxFrequency"])
        pyHDB.set_wave_frequencies()  # Definition of omega.

        # Wave direction discretization.

        wave_direction_path = discretization_path + "/WaveDirections"

        pyHDB.nb_wave_dir = np.array(reader[wave_direction_path + "/NbWaveDirections"])
        pyHDB.min_wave_dir = np.array(reader[wave_direction_path + "/MinAngle"]) # Deg.
        pyHDB.max_wave_dir = np.array(reader[wave_direction_path + "/MaxAngle"]) # Deg.
        pyHDB.set_wave_directions() # Definition of beta in rad.

        # Time sample.

        time_path = discretization_path + "/Time"

        pyHDB.nb_time_samples = np.array(reader[time_path + "/NbTimeSample"])
        final_time = np.array(reader[time_path + "/FinalTime"])
        pyHDB.dt = np.array(reader[time_path + "/TimeStep"])
        pyHDB.time = np.arange(start=0., stop=final_time + pyHDB.dt, step=pyHDB.dt) # Definition of time.

    def read_mesh(self, reader, mesh_path):

        """This function reads the mesh quantities of the *.hdb5 file.

        Parameters
        ----------
        reader : string
            *.hdb5 file.
        mesh_path : string
            Path to the mesh folder.
        """

        # Mesh data.
        nb_vertices = np.array(reader[mesh_path + "/NbVertices"])
        vertices = np.array(reader[mesh_path + "/Vertices"])
        nb_faces = np.array(reader[mesh_path + "/NbFaces"])
        faces = np.array(reader[mesh_path + "/Faces"])
        mesh = Mesh(vertices, faces)

        # Verification of mesh information consistency
        assert nb_vertices == mesh.nb_vertices
        assert nb_faces == mesh.nb_faces

        return mesh

    def read_mode(self, reader, body, ForceOrMotion, body_modes_path):
        """This function reads the force and motion modes of the *.hdb5 file.

        Parameters
        ----------
        reader : string
            *.hdb5 file.
        body : BodyDB.
            Body.
        ForceOrMotion : int
            0 for Force, 1 for Motion.
        body_modes_path : string
            Path to body modes.
        """

        for iforce in range(0,6):
            if (ForceOrMotion == 0):  # Force.
                mode_path = body_modes_path + "/ForceModes/Mode_%u" % iforce
            else:  # Motion.
                mode_path = body_modes_path + "/MotionModes/Mode_%u" % iforce

            if (iforce >= 3):
                body.point = np.array(reader[mode_path + "/Point"])

    def read_mask(self, reader, body, mask_path):
        """This function reads the Force and Motion masks into the *.hdb5 file.

        Parameters
        ----------
        reader : string
            *.hdb5 file.
        body : BodyDB.
            Body.
        mask_path : string
            Path to the masks.
        """

        body.Motion_mask = np.array(reader[mask_path + "/MotionMask"])
        body.Force_mask = np.array(reader[mask_path + "/ForceMask"])

    def read_excitation(self, reader, pyHDB, body, excitation_path):

        """This function reads the diffraction and Froude-Krylov loads into the *.hdb5 file.

        Parameters
        ----------
        reader : string
            *.hdb5 file.
        pyHDB : object
            pyHDB object for storing the hydrodynamic database.
        body : BodyDB.
            Body.
        excitation_path : string
            Path to excitation loads.
        """

        # Froude-Krylov loads;

        fk_path = excitation_path + "/FroudeKrylov"

        for idir in range(0, pyHDB.nb_wave_dir):

            wave_dir_path = fk_path + "/Angle_%u" % idir

            # Check of the wave direction.
            assert pyHDB.wave_dir[idir] == np.radians(np.array(reader[wave_dir_path + "/Angle"]))

            # Real parts.
            body.Froude_Krylov[:, :, idir].real = np.array(reader[wave_dir_path + "/RealCoeffs"])

            # Imaginary parts.
            body.Froude_Krylov[:, :, idir].imag = np.array(reader[wave_dir_path + "/ImagCoeffs"])

        # Diffraction loads.

        diffraction_path = excitation_path + "/Diffraction"

        for idir in range(0, pyHDB.nb_wave_dir):

            wave_dir_path = diffraction_path + "/Angle_%u" % idir

            # Check of the wave direction.
            assert pyHDB.wave_dir[idir] == np.radians(np.array(reader[wave_dir_path + "/Angle"]))

            # Real parts.
            body.Diffraction[:, :, idir].real = np.array(reader[wave_dir_path + "/RealCoeffs"])

            # Imaginary parts.
            body.Diffraction[:, :, idir].imag = np.array(reader[wave_dir_path + "/ImagCoeffs"])

    def read_radiation(self, reader, pyHDB, body, radiation_path):

        """This function reads the added mass and damping coefficients and the impulse response functions with and without forward speed of the *.hdb5 file.

        Parameters
        ----------
        Writer : string
            *.hdb5 file.
        pyHDB : object
            pyHDB object for storing the hydrodynamic database.
        body : BodyDB.
            Body.
        radiation_path : string
            Path to radiation loads.
        """

        # Initializations.
        body.Inf_Added_mass = np.zeros((6, 6 * pyHDB.nb_bodies), dtype=np.float)
        body.irf = np.zeros((6, 6 * pyHDB.nb_bodies, pyHDB.nb_time_samples), dtype=np.float)
        body.irf_ku = np.zeros((6, 6 * pyHDB.nb_bodies, pyHDB.nb_time_samples), dtype=np.float)

        for j in range(pyHDB.nb_bodies):

            # Paths.
            radiation_body_motion_path = radiation_path + "/BodyMotion_%u" % j
            added_mass_path = radiation_body_motion_path + "/AddedMass"
            radiation_damping_path = radiation_body_motion_path + "/RadiationDamping"
            irf_path = radiation_body_motion_path + "/ImpulseResponseFunctionK"
            irf_ku_path = radiation_body_motion_path + "/ImpulseResponseFunctionKU"

            # Infinite added mass.
            body.Inf_Added_mass[:,6*j:6*(j+1)] = np.array(reader[radiation_body_motion_path + "/InfiniteAddedMass"])

            for iforce in range(0, 6):

                # Added mass.
                body.Added_mass[:, 6*j+iforce, :] = np.array(reader[added_mass_path + "/DOF_%u" % iforce])

                # Damping.
                body.Damping[:, 6 * j + iforce, :] = np.array(reader[radiation_damping_path + "/DOF_%u" % iforce])

                # Impulse response functions without forward speed.
                body.irf[:, 6*j+iforce, :] = np.array(reader[irf_path + "/DOF_%u" % iforce])

                # Impulse response functions with forward speed.
                body.irf_ku[:, 6*j+iforce, :] = np.array(reader[irf_ku_path + "/DOF_%u" % iforce])

    def read_hydrostatic(self, reader, body, hydrostatic_path):

        """This function reads the hydrostatic stiffness matrix into the *.hdb5 file.

        Parameters
        ----------
        reader : string
            *.hdb5 file.
        body : BodyDB.
            Body.
        hydrostatic_path : string
            Path to hydrostatic stiffness matrix.
        """

        try:
            reader[hydrostatic_path + "/StiffnessMatrix"]
            body.activate_hydrostatic()
            body.hydrostatic.matrix = np.array(reader[hydrostatic_path + "/StiffnessMatrix"])
        except:
            pass

    def read_mass_matrix(self, reader, body, inertia_path):

        """This function reads the mass matrix matrix into the *.hdb5 file.

        Parameters
        ----------
        reader : string
            *.hdb5 file.
        body : BodyDB.
            Body.
        inertia_path : string
            Path to inertia matrix.
        """

        try:
            reader[inertia_path + "/InertiaMatrix"]
            body.activate_inertia()
            body.inertia.matrix = np.array(reader[inertia_path + "/InertiaMatrix"])
        except:
            pass

    def read_RAO(self, reader, pyHDB, body, RAO_path):

        """This function reads the RAO into the *.hdb5 file.

        Parameters
        ----------
        reader : string
            *.hdb5 file.
        pyHDB : object
            pyHDB object for storing the hydrodynamic database.
        body : BodyDB.
            Body.
        excitation_path : string
            Path to excitation loads.
        """

        # Definition.
        body.RAO = np.zeros((6, pyHDB.nb_wave_freq, pyHDB.nb_wave_dir), dtype=np.complex)

        try:

            for idir in range(0, pyHDB.nb_wave_dir):

                wave_dir_path = RAO_path + "/Angle_%u" % idir

                # Check of the wave direction.
                assert pyHDB.wave_dir[idir] == np.radians(np.array(reader[wave_dir_path + "/Angle"]))

                # Amplitude.
                Abs_RAO = np.array(reader[wave_dir_path + "/Amplitude"])

                # Phase.
                Phase_RAO = np.radians(np.array(reader[wave_dir_path + "/Phase"]))

                # RAO.
                body.RAO[:, :, idir] = Abs_RAO * np.exp(1j * Phase_RAO)
                pyHDB.has_RAO = True # Written for each body but it does not matter.

        except:
            pass

    def read_bodies(self, reader, pyHDB):
        """This function reads the body data of the *.hdb5 file.

        Parameters
        ----------
        reader : string.
            *.hdb5 file.
        pyHDB : object
            pyHDB object for storing the hydrodynamic database.
        """

        for ibody in xrange(pyHDB.nb_bodies):

            body_path = '/Bodies/Body_%u' % ibody

            # Index of the body.
            id = np.array(reader[body_path + "/ID"])
            assert ibody == id

            # Mesh.
            mesh = self.read_mesh(reader, body_path + "/Mesh")

            # Body definition.
            body = BodyDB(id, pyHDB.nb_bodies, pyHDB.nb_wave_freq, pyHDB.nb_wave_dir, mesh)

            # Force modes.
            self.read_mode(reader, body, 0, body_path + "/Modes")

            # Motion modes.
            self.read_mode(reader, body, 1, body_path + "/Modes")

            # Masks.
            self.read_mask(reader, body, body_path + "/Mask")

            # Diffraction and Froude-Krylov loads.
            self.read_excitation(reader, pyHDB, body, body_path + "/Excitation")

            # Added mass and damping coefficients and impulse response functions.
            self.read_radiation(reader, pyHDB, body, body_path + "/Radiation")

            # Hydrostatics.
            self.read_hydrostatic(reader, body, body_path + "/Hydrostatic")

            # Mass matrix.
            self.read_mass_matrix(reader, body, body_path + "/Inertia")

            # RAO.
            self.read_RAO(reader, pyHDB, body, body_path + "/RAO")

            # Add body to pyHDB.
            pyHDB.append(body)

    def read_wave_drift(self, reader, pyHDB, wave_drift_path):
        """This function writes the wave drift loads into the *.hdb5 file.

        Parameters
        ----------
        reader : string
            *.hdb5 file.
        pyHDB : object
            pyHDB object for storing the hydrodynamic database.
        wave_drift_path : string, optional
            Path to wave drift loads.
        """

        try:
            reader[wave_drift_path]
            pyHDB._wave_drift = WaveDriftDB()

            # sym_x.
            if(int(np.array(reader[wave_drift_path + "/sym_x"])) == 0):
                pyHDB._wave_drift.sym_x = False
            else:
                pyHDB._wave_drift.sym_x = True

            # sym_y
            if (int(np.array(reader[wave_drift_path + "/sym_y"])) == 0):
                pyHDB._wave_drift.sym_y = False
            else:
                pyHDB._wave_drift.sym_y = True

            # Wave frequencies.
            pyHDB._wave_drift.discrete_frequency = np.array(reader[wave_drift_path + "/freq"])

            # Modes.
            for mode in ["/surge", "/sway", "/heave", "/roll", "/pitch", "/yaw"]:
                try:
                    reader[wave_drift_path + mode]

                    # Loop over the wave directions.
                    for ibeta in range(0, pyHDB.nb_wave_dir):

                        # Path.
                        heading_path = wave_drift_path + mode + "/heading_%u" % ibeta

                        # Check wave direction.
                        assert pyHDB.wave_dir[ibeta] == np.array(reader[heading_path + "/heading"])

                        # Wave drift coefficients.
                        if(mode == "/surge"):
                            pyHDB._wave_drift.add_cx(pyHDB._wave_drift.discrete_frequency, list(reader[heading_path + "/data"]), pyHDB.wave_dir[ibeta])
                        elif(mode == "/sway"):
                            pyHDB._wave_drift.add_cy(pyHDB._wave_drift.discrete_frequency, list(reader[heading_path + "/data"]), pyHDB.wave_dir[ibeta])
                        elif(mode == "/heave"):
                            pyHDB._wave_drift.add_cz(pyHDB._wave_drift.discrete_frequency, list(reader[heading_path + "/data"]), pyHDB.wave_dir[ibeta])
                        elif(mode == "/roll"):
                            pyHDB._wave_drift.add_cr(pyHDB._wave_drift.discrete_frequency, list(reader[heading_path + "/data"]), pyHDB.wave_dir[ibeta])
                        elif(mode == "/pitch"):
                            pyHDB._wave_drift.add_cm(pyHDB._wave_drift.discrete_frequency, list(reader[heading_path + "/data"]), pyHDB.wave_dir[ibeta])
                        else:
                            pyHDB._wave_drift.add_cn(pyHDB._wave_drift.discrete_frequency, list(reader[heading_path + "/data"]), pyHDB.wave_dir[ibeta])

                except:
                    pass

        except:
            pass

    def read_version(self, reader, pyHDB):
        """This function reads the version of the *.hdb5 file.

        Parameter
        ---------
        reader : string
            *.hdb5 file.
        pyHDB : object
            pyHDB object for storing the hydrodynamic database.
        """

        # Version.
        try:
            pyHDB.version = np.array(reader['Version'])
        except:
            pyHDB.version = 1.0













































