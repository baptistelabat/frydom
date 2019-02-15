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

"""Module to create a body database for frydom hydrodynamic database"""

import numpy as np

import sys
import hydro_db
from hydrostatic_db import HydrostaticDB
from wave_drift_db import WaveDriftDB
from discretization_db import DiscretizationDB
import matplotlib.pyplot as plt
from scipy import interpolate


class BodyDB(object):

    """
        Class for writing the body data into the *.hdb5 file.
    """

    def __init__(self, hdb, i_body):

        """
        Constructor of the class BodyDB.

        Parameters
        ----------
        hdb : HydroDB
            Hydrodynamic database.
        i_body : int
            Index of the body.
        """

        self._i_body = None
        self._hdb = None
        self._discretization = None
        self._wave_drift = None
        self._hydrostatic = None
        self.load_data(hdb, i_body)
        self._position = np.zeros(3, dtype=np.float)

    @property
    def name(self):

        """This subroutine gives the name of the mesh of a body.

        Returns
        -------
        string
            Name of the mesh of a body.
        """

        return self._hdb.body_mapper.body[self.id].mesh.name

    @property
    def id(self):

        """This subroutine gives the index of a body.

        Returns
        -------
        int
            Index of a body.
        """

        return self._i_body

    @property
    def position(self):

        """This subroutine gives the position of a body.

        Returns
        -------
        Array of floats
            Position of a body.
        """

        return self._position

    @property
    def force_modes(self):

        """This subroutine gives the force modes of a body.
        """

        return self._hdb.body_mapper.body[self.id].force_modes

    @property
    def nb_force_modes(self):

        """This subroutine gives the number of force modes of a body.

        Returns
        -------
        int
            Number of force modes.
        """

        return self._hdb.body_mapper.body[self.id].nb_force_modes

    @property
    def motion_modes(self):

        """This subroutine gives the motion modes of a body.
        """

        return self._hdb.body_mapper.body[self.id].motion_modes

    @property
    def nb_dof(self):

        """This subroutine gives the number of dof of a body.

        Returns
        -------
        int
            Number of dof.
        """

        return self._hdb.body_mapper.body[self.id].nb_dof

    @property
    def nb_vertices(self):

        """This subroutine gives the number of vertices in the body mesh.

        Returns
        -------
        int
            Number of vertices in the body mesh.
        """

        return self._hdb.body_mapper.body[self.id].mesh.nb_vertices

    @property
    def vertices(self):

        """This subroutine gives the vertices of the body mesh.

        Returns
        -------
        Array of floats
            Vertices in the body mesh.
        """

        return self._hdb.body_mapper.body[self.id].mesh.vertices

    @property
    def nb_faces(self):

        """This subroutine gives the number of faces in the body mesh.

        Returns
        -------
        int
            Number of faces in the body mesh.
        """

        return self._hdb.body_mapper.body[self.id].mesh.nb_faces

    @property
    def faces(self):

        """This subroutine gives the faces of the body mesh.

        Returns
        -------
        Array of int
            Faces in the body mesh.
        """

        return self._hdb.body_mapper.body[self.id].mesh.faces

    @property
    def nb_bodies(self):

        """This subroutine gives the number of bodies.

        Returns
        -------
        int
            Number of bodies.
        """

        return self._hdb.body_mapper.nb_bodies

    @property
    def wave_dirs(self):

        """This subroutine gives the number of wave directions.

        Returns
        -------
        int
            Number of wave directions
        """

        return self._discretization.wave_dirs

    @property
    def diffraction(self):

        """This subroutine gives the diffraction loads of the body.

        Returns
        -------
        Array of floats
            Diffraction loads of the wave directions.
        """

        i_start = self._hdb.body_mapper.get_general_force_index(self.id, 0)
        nb_force = self._hdb.body_mapper.body[self.id].nb_force_modes
        i_end = self._hdb.body_mapper.get_general_force_index(self.id, nb_force-1)
        return self._hdb.diffraction_db.data[i_start:i_end+1, :, :]

    @property
    def froude_krylov(self):

        """This subroutine gives the Froude-Krylov loads of the body.

        Returns
        -------
        Array of floats
            Froude-Krylov loads of the body.
        """

        i_start = self._hdb.body_mapper.get_general_force_index(self.id, 0)
        nb_force = self._hdb.body_mapper.body[self.id].nb_force_modes
        i_end = self._hdb.body_mapper.get_general_force_index(self.id, nb_force-1)
        return self._hdb.froude_krylov_db.data[i_start:i_end+1, :, :]

    @property
    def hydrostatic(self):

        """This subroutine gives the hydrostatic data of the body.

        Returns
        -------
        HydrostaticDB
            Hydrostatic data of the body.
        """

        return self._hydrostatic

    @property
    def wave_drift(self):

        """This subroutine gives the wave drift data of the body.

        Returns
        -------
        WaveDriftDB
            Wave drift data of the body.
        """

        return self._wave_drift

    @property
    def discretization(self):

        """This subroutine gives the discretization parameters for the body.

        Returns
        -------
        DiscretizationDB
            Wave drift data of the body.
        """

        return self._discretization

    @discretization.setter
    def discretization(self, value):

        """This subroutine sets the discretization parameters for the body.

        Parameter
        ----------
        DiscretizationDB : value
            Discretization parameters for the body.
        """

        if isinstance(value, DiscretizationDB):
            self._discretization = value
        else:
            print("warning : wrong type value")

    def added_mass(self, i_body_motion):

        """This subroutine gives the added mass matrices for the body.

        Parameter
        ----------
        int : i_body_motion
            Index of the body.

        Returns
        -------
        np.ndarray
            Added mass matrices.
        """

        return self._hdb.radiation_db.get_added_mass(self.id, i_body_motion)

    def infinite_added_mass(self, i_body_motion):

        """This subroutine gives the infinite added mass matrix for the body.

        Parameter
        ----------
        int : i_body_motion
            Index of the body.

        Returns
        -------
        np.ndarray
            Infinite added mass matrix.
        """

        return self._hdb.radiation_db.get_infinite_added_mass_matrix(self.id, i_body_motion)

    def radiation_damping(self, i_body_motion):

        """This subroutine gives the damping matrices for the body.

        Parameter
        ----------
        int : i_body_motion
            Index of the body.

        Returns
        -------
        np.ndarray
            Damping matrices.
        """

        return self._hdb.radiation_db.get_radiation_damping(self.id, i_body_motion)

    def irf_k(self, i_body_motion):

        """This subroutine gives the impulse response functions for the body.

        Parameter
        ----------
        int : i_body_motion
            Index of the body.

        Returns
        -------
        np.ndarray
            Impulse response functions.
        """

        return self._hdb.radiation_db.get_irf_db().get_impulse_response(self.id, i_body_motion)

    def irf_ku(self, i_body_motion):

        """This subroutine gives the impulse response function relative to the ship advance speed.

        Parameter
        ----------
        int : i_body_motion
            Index of the body.

       Returns
       -------
       np.ndarray
           Impulse reponse function relative to the ship advance speed.
       """

        return self._hdb.radiation_db.get_irf_ku().get_impulse_response(self.id, i_body_motion)

    def cutoff_scaling_irf_k(self, tc, i_body_motion, i_force, i_dof, auto_apply=False):

        """This subroutine plots the effect of the filter about the impule response function.

        Parameters
        ----------
        float : tc
            Cutting time.
        int : i_body_motion
            Index of the body.
        int : i_force
            Index of the index of the force of the current body.
        int : i_dof
            Index of the dof of the moving body.
        Bool : auto_apply, optional
            Automatic application of the filtering, not if flase (default).
       """

        time = self.discretization.time

        try:
            coeff = np.exp(-9.*time*time / (tc*tc))
        except:
            coeff = np.zeros(time.size)

        bool = False

        if auto_apply:
            bool = True
        else:
            plt.figure()
            plt.plot(time, self.irf_k(i_body_motion)[i_force, i_dof, :], label="initial")
            plt.plot(time, self.irf_k(i_body_motion)[i_force, i_dof, :] * coeff, label="with scaling factor")
            plt.xlabel("time (s)")
            plt.ylabel("IRF")
            plt.legend()
            plt.show()

            # raw_input returns the empty string for "enter"
            yes = {'yes', 'y', 'ye', ''}
            no = {'no', 'n'}

            choice = raw_input("Apply scaling (y/n) ?").lower()
            if choice in yes:
                bool = True
            elif choice in no:
                bool = False
            else:
                sys.stdout.write("Please respond with 'yes' or 'no'")

        if bool:
            self.irf_k(i_body_motion)[i_force, i_dof, :] *= coeff

        return

    def cut_off_scaling_irf_ku(self, tc, i_body_motion, i_force, i_dof, auto_apply=False):

        """This subroutine plots the effect of the filter about the impule response function relative to the ship advance speed.

        Parameters
        ----------
        int : i_body_motion
            Index of the body.
        int : i_force
            Force mode.
        int : i_dof
            Dof.
        Bool : auto_apply, optional
            Automatic application of the filtering, not if flase (default).
       """

        time = self.discretization.time

        try:
            coeff = np.exp(-9.*time*time / (tc*tc))
        except:
            coeff = np.zeros(time.size)

        bool = False
        if auto_apply:
            bool = True
        else:
            plt.figure()
            plt.plot(time, self.irf_ku(i_body_motion)[i_force, i_dof, :], label="initial")
            plt.plot(time, self.irf_ku(i_body_motion)[i_force, i_dof, :] * coeff, label="with scaling factor")
            plt.xlabel("time (s)")
            plt.ylabel("IRF")
            plt.legend()
            plt.show()

            # raw_input returns the empty string for "enter"
            yes = {'yes', 'y', 'ye', ''}
            no = {'no', 'n'}

            choice = raw_input("Apply scaling (y/n) ?").lower()
            if choice in yes:
                bool = True
            elif choice in no:
                bool = False
            else:
                sys.stdout.write("Please respond with 'yes' or 'no'")

        if bool:
            self.irf_ku(i_body_motion)[i_force, i_dof, :] *= coeff

        return

    def load_data(self, hdb, i_body):

        """This subroutine loads the hydrodynamic database.

        Parameters
        ----------
        HydroDB : hdb
            Hydrodynamic database.
        int : i_body
            Index of the body.
       """

        self._i_body = i_body
        self._hdb = hdb
        return

    def activate_hydrostatic(self):

        """This subroutine initializes the hydrostatic parameters.
        """

        self._hydrostatic = HydrostaticDB()

    def activate_wave_drift(self):

        """This subroutine initializes the wave drift force parameters.
        """

        self._wave_drift = WaveDriftDB()

    def write_hdb5(self, writer):

        """This subroutine writes the body data into the *.hdb5 file.

        Parameters
        ----------
        Writer : string
            *.hdb5 file.
        """

        body_path = '/Bodies/Body_%u' % self.id

        dset = writer.create_group(body_path)

        # Body name
        dset = writer.create_dataset(body_path + "/BodyName", data=self.name)
        dset.attrs['Description'] = "Body name"

        # Id of the body
        dset = writer.create_dataset(body_path + "/ID", data=self.id)
        dset.attrs['Description'] = "Body identifier"

        # Position of the body
        dset = writer.create_dataset(body_path + "/BodyPosition", data=self.position)
        dset.attrs['Description'] = "Position of the body in the absolute frame"

        # Modes Force
        self.write_mode_force(writer, body_path + "/Modes")

        # Modes Motion
        self.write_mode_motion(writer, body_path + "/Modes")

        # Mesh file
        self.write_mesh(writer, body_path + "/Mesh")

        # Excitation force
        self.write_excitation(writer, body_path + "/Excitation")

        # Radiation force
        self.write_radiation(writer, body_path + "/Radiation")

        # Wave drift coefficients
        if self._wave_drift:
            self.write_wave_drift(writer, body_path + "/WaveDrift")

        # Hydrostatic stiffness matrix
        if self._hydrostatic:
            self.write_hydrostatic(writer, body_path + "/Hydrostatic")

        return

    def write_mode_force(self, writer, body_modes_path="/Modes"):

        """This subroutine writes the force modes into the *.hdb5 file.

        Parameters
        ----------
        Writer : string
            *.hdb5 file.
        body_modes_path : string, optional
            Path to body modes.
        """

        dset = writer.create_dataset(body_modes_path + "/NbForceModes", data=self.nb_force_modes)
        dset.attrs['Description'] = "Number of force modes for body number %u" % self.id

        for imode, force_mode in enumerate(self.force_modes):
            mode_path = body_modes_path + "/ForceModes/Mode_%u" % imode
            writer.create_group(mode_path)
            writer.create_dataset(mode_path + "/Direction", data=force_mode.direction)

            if isinstance(force_mode, hydro_db.ForceMode):
                writer.create_dataset(mode_path + "/Type", data='LINEAR')

            elif isinstance(force_mode, hydro_db.MomentMode):
                writer.create_dataset(mode_path + "/Type", data='ANGULAR')
                writer.create_dataset(mode_path + "/Point", data=force_mode.point)

        return

    def write_mode_motion(self, writer, body_modes_path="/Modes"):

        """This subroutine writes the motion modes into the *.hdb5 file.

        Parameters
        ----------
        Writer : string
            *.hdb5 file.
        body_modes_path : string, optional
            Path to body modes.
        """

        dset = writer.create_dataset(body_modes_path + "/NbMotionModes", data=self.nb_dof)
        dset.attrs['Description'] = "Number of motion modes for body number %u" % self.id

        for idof, motion_mode in enumerate(self.motion_modes):
            mode_path = body_modes_path + "/MotionModes/Mode_%u" % idof
            writer.create_group(mode_path)
            writer.create_dataset(mode_path + "/Direction", data=motion_mode.direction)

            if isinstance(motion_mode, hydro_db.TranslationMode):
                writer.create_dataset(mode_path + "/Type", data="LINEAR")

            elif isinstance(motion_mode, hydro_db.RotationMode):
                writer.create_dataset(mode_path + "/Type", data="ANGULAR")
                writer.create_dataset(mode_path + "/Point", data=motion_mode.point)

        return

    def write_mesh(self, writer, mesh_path="/Mesh"):

        """This subroutine writes the mesh quantities into the *.hdb5 file.

        Parameters
        ----------
        Writer : string
            *.hdb5 file.
        """

        writer.create_dataset(mesh_path + "/NbVertices", data=self.nb_vertices)
        writer.create_dataset(mesh_path + "/Vertices", data=self.vertices)
        writer.create_dataset(mesh_path + "/NbFaces", data=self.nb_faces)
        writer.create_dataset(mesh_path + "/Faces", data=self.faces)

        return

    def write_excitation(self, writer, excitation_path="/Excitation"):

        """This subroutine writes the excitation loads into the *.hdb5 file.

        Parameters
        ----------
        Writer : string
            *.hdb5 file.
        excitation_path : string, optional
            Path to excitation loads.
        """

        # Froude-Krylov excitation

        fk_path = excitation_path + "/FroudeKrylov"
        writer.create_group(fk_path)

        for idir, wave_dir in enumerate(self.wave_dirs):

            wave_dir_path = fk_path + "/Angle_%u" % idir
            writer.create_group(wave_dir_path)

            dset = writer.create_dataset(wave_dir_path + "/Angle", data=wave_dir)
            dset.attrs['Unit'] = 'deg'
            dset.attrs['Description'] = "Wave direction angle of the data"

            dset = writer.create_dataset(wave_dir_path + "/RealCoeffs", data=self.froude_krylov[:, :, idir].real)
            dset.attrs['Unit'] = ''
            dset.attrs['Description'] = "Real part of the Froude-Krylov hydrodynamic coefficients for %u forces " \
                                        "on body %u as a function of frequency" % \
                                        (self.nb_force_modes, self.id)

            dset = writer.create_dataset(wave_dir_path + "/ImagCoeffs", data=self.froude_krylov[:, :, idir].imag)
            dset.attrs['Unit'] = ''
            dset.attrs['Description'] = "Imaginary part of the Froude-Krylov hydrodynamic coefficients for %u " \
                                        "forces on body %u as a function of frequency" % (
                                        self.nb_force_modes, self.id)

        # Diffraction excitation

        diffraction_path = excitation_path + "/Diffraction"
        writer.create_group(diffraction_path)

        for idir, wave_dir in enumerate(self.wave_dirs):

            wave_dir_path = diffraction_path + "/Angle_%u" % idir
            writer.create_group(wave_dir_path)

            writer.create_dataset(wave_dir_path + "/Angle", data=wave_dir)
            dset.attrs['Unit'] = 'deg'
            dset.attrs['Description'] = "Wave direction angle of the data"

            writer.create_dataset(wave_dir_path + "/RealCoeffs", data=self.diffraction[:, :, idir].real)
            dset.attrs['Unit'] = ''
            dset.attrs['Description'] = "Real part of the diffraction hydrodynamic coefficients for %u forces " \
                                        "on body %u as a function of frequency" % \
                                        (self.nb_force_modes, self.id)

            writer.create_dataset(wave_dir_path + "/ImagCoeffs", data=self.diffraction[:, :, idir].imag)
            dset.attrs['Unit'] = ''
            dset.attrs['Description'] = "Imaginary part of the diffraction hydrodynamic coefficients for %u forces " \
                                        "on body %u as a function of frequency" % \
                                        (self.nb_force_modes, self.id)

        return

    def write_radiation(self, writer, radiation_path="/Radiation"):

        """This subroutine writes the radiation coefficients and impulse response functions into the *.hdb5 file.

        Parameters
        ----------
        Writer : string
            *.hdb5 file.
        radiation_path : string, optional
            Path to radiation loads.
        """

        writer.create_group(radiation_path)

        for j in range(self.nb_bodies):

            radiation_body_motion_path = radiation_path + "/BodyMotion_%u" % j

            dg = writer.create_group(radiation_body_motion_path)
            dg.attrs['Description'] = "Hydrodynamic coefficients for motion of body %u that radiates waves and " \
                                      " generate force on body %u" % (j, self.id)

            added_mass_path = radiation_body_motion_path + "/AddedMass"
            dg = writer.create_group(added_mass_path)
            dg.attrs['Description'] = "Added mass coefficients for acceleration of body %u that radiates waves " \
                                      "and generates forces on body %u" % (j, self.id)

            radiation_damping_path = radiation_body_motion_path + "/RadiationDamping"
            dg = writer.create_group(radiation_damping_path)
            dg.attrs['Description'] = "Wave damping coefficients for velocity of body %u that radiates waves " \
                                      "and generates forces on body %u" % (j, self.id)

            irf_path = radiation_body_motion_path + "/ImpulseResponseFunctionK"
            dg = writer.create_group(irf_path)
            dg.attrs['Description'] = "Impulse response function for velocity of body %u that radiates waves " \
                                      "and generates forces on body %u" % (j, self.id)

            irf_ku_path = radiation_body_motion_path + "/ImpulseResponseFunctionKU"
            dg = writer.create_group(irf_ku_path)
            dg.attrs['Description'] = "Impulse response function Ku for velocity of body %u that radiates waves " \
                                      "and generates forces on body %u" % (j, self.id)

            dset = writer.create_dataset(radiation_body_motion_path + "/InfiniteAddedMass",
                                         data=self.infinite_added_mass(j))
            dset.attrs['Description'] = "Infinite added mass matrix that modifies the apparent mass of body %u from " \
                                        "acceleration of body %u" % (self.id, j)

            added_mass = self.added_mass(j)
            radiation_damping = self.radiation_damping(j)
            irf = self.irf_k(j)
            irf_ku = self.irf_ku(j)

            for imode in range(self.nb_dof):

                # Added mass
                dset = writer.create_dataset(added_mass_path + "/DOF_%u" % imode, data=added_mass[:, :, imode])
                dset.attrs['Unit'] = ""
                dset.attrs['Description'] = "Added mass coefficients for an acceleration of body %u and force on " \
                                            "body %u (nbForce x nw)" % (j, self.id)

                # Wave damping
                dset = writer.create_dataset(radiation_damping_path + "/DOF_%u" % imode,
                                        data=radiation_damping[:, :, imode])
                dset.attrs['Unit'] = ""
                dset.attrs['Description'] = "Wave damping coefficients for an acceleration of body %u and force " \
                                            "on body %u (nbForce x nw)" % (j, self.id)

                # Impulse response function
                dset = writer.create_dataset(irf_path + "/DOF_%u" % imode,
                                        data=irf[:, imode, :])
                dset.attrs['Description'] = "Impulse response functions"

                # Impulse response function Ku
                dset = writer.create_dataset(irf_ku_path + "/DOF_%u" % imode,
                                        data=irf_ku[:, imode, :])
                dset.attrs['Description'] = "Impulse response functions Ku"

        return

    def write_hydrostatic(self, writer, hydrostatic_path="/Hydrostatic"):

        """This subroutine writes the hydrostatic stiffness matrix into the *.hdb5 file.

        Parameters
        ----------
        Writer : string
            *.hdb5 file.
        hydrostatic_path : string, optional
            Path to hydrostatic stiffness matrix.
        """

        dg = writer.create_group(hydrostatic_path)

        dset = dg.create_dataset(hydrostatic_path + "/StiffnessMatrix", data=self.hydrostatic.matrix)
        dset.attrs['Description'] = "Hydrostatic stiffness matrix at the Center of gravity of the body"

        return

    def write_wave_drift(self, writer, wave_drift_path="/WaveDrift"):

        """This subroutine writes the wave drift loads into the *.hdb5 file.

        Parameters
        ----------
        Writer : string
            *.hdb5 file.
        wave_drift_path : string, optional
            Path to wave drift loads.
        """

        dg = writer.create_group(wave_drift_path)

        for key, mode in self.wave_drift.modes.items():
            grp_modes = dg.require_group(mode.name)

            for i_angle, angle in enumerate(mode.heading):
                grp_dir = grp_modes.require_group("heading_%i" % i_angle)

                # Set heading angle
                dset = grp_dir.create_dataset("heading", data=angle)
                dset.attrs['Unit'] = 'rad'
                dset.attrs['Description'] = "Heading angle"

                # Set data
                dset = grp_dir.create_dataset("data", data=mode.data[i_angle, :])
                dset.attrs['Description'] = "Wave Drift force coefficient"

        # Set frequency
        dset = dg.create_dataset("freq", data=self.wave_drift.discrete_frequency)
        dset.attrs['Unit'] = "rads"
        dset.attrs['Description'] = "Time discretization of the data"

        # Set sym
        dset = dg.create_dataset("sym_x", data=self.wave_drift.sym_x)
        dset.attrs['Description'] = "Symmetry along x"
        dset = dg.create_dataset('sym_y', data=self.wave_drift.sym_y)
        dset.attrs['Description'] = "Symmetry along y"


