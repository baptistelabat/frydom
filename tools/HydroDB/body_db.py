#!/usr/bin/env python
#  -*- coding: utf-8 -*-
"""Module to create a body database for frydom hydrodynamic database"""

import numpy as np
from HydroDB import hydro_db
from HydroDB.hydrostatic_db import HydrostaticDB
from HydroDB.wave_drift_db import WaveDriftDB

class body_db(object):

    def __init__(self):
        self._ibody = None
        self._hdb = None
        self._wave_drift = None
        self._hydrostatic = None
        self._wave_dirs = None

    @property
    def name(self):
        return self._hdb.body_mapper.body[self.id].name

    @property
    def id(self):
        return self._ibody

    @property
    def position(self):
        return self._hdb.body_mapper.body[self.id].position

    @property
    def force_modes(self):
        return self._hdb.body_mapper.body[self.id].force_modes

    @property
    def nb_force_modes(self):
        return self._hdb.body_mapper.body[self.id].nb_force_modes

    @property
    def motion_modes(self):
        return self._hdb.body_mapper.body[self.id].motion_modes

    @property
    def nb_dof(self):
        return self._hdb.body_mapper.body[self.id].nb_dof

    @property
    def nb_vertices(self):
        return self._hdb.body_mapper.body[self.id].mesh.nb_vertices

    @property
    def vertices(self):
        return self._hdb.body_mapper.body[self.id].mesh.vertices

    @property
    def nb_faces(self):
        return self._hdb.body_mapper.body[self.id].mesh.nb_faces

    @property
    def faces(self):
        return self._hdb.body_mapper.body[self.id].mesh.faces

    @property
    def nb_bodies(self):
        return self._hdb.body_mapper.nb_bodies

    @property
    def wave_dirs(self):
        return self._wave_dirs

    @wave_dirs.setter
    def wave_dirs(self, value):
        self._wave_dirs = value

    @property
    def diffraction(self):
        i_start = self._hdb.body_mapper.get_general_force_index(self.id, 0)
        nb_force = self._hdb.boddy_mapper.body[self.id].nb_forces
        i_end = self._hdb.body_mapper.get_general_force_index(self.id, nb_force-1)
        return self._hdb.diffraction_db[i_start:i_end+1, :, :]

    @property
    def froude_krylov(self):
        i_start = self._hdb.body_mapper.get_general_force_index(self.id, 0)
        nb_force = self._hdb.boddy_mapper.body[self.id].nb_forces
        i_end = self._hdb.body_mapper.get_general_force_index(self.id, nb_force-1)
        return self._hdb.froude_krylov_db[i_start:i_end+1, :, :]

    @property
    def hydrostatic(self):
        return self._hydrostatic

    @property
    def wave_drift(self):
        return self._wave_drift

    def added_mass(self, i_body_motion):
        return self._hdb.get_added_mass(self.id, i_body_motion)

    def radiation_damping(self, i_body_motion):
        return self._hdb.get_radiation_damping(self.id, i_body_motion)

    def irf_k(self, i_body_motion):
        return self._hdb.get_impulse_response(self.id, i_body_motion)

    def irf_ku(self, i_body_motion):
        return self._hdb.get_impulse_response_ku(self.id, i_body_motion)

    def load_data(self, hdb, ibody):
        self._ibody = ibody
        self._hdb = hdb
        self._hydrostatic = HydrostaticDB()
        self._wave_drift = WaveDriftDB()
        return

    def write_hdb5(self, writer):

        # Body name
        dset = writer.create_group("/BodyName", data=self.name)
        dset.attrs['Description'] = "Body name"

        # Id of the body
        dset = writer.create_dataset("/ID", data=self.id)
        dset.attrs['Description'] = "Body identifier"

        # Position of the body
        dset = writer.create_group("/BodyPosition", data=self.position)
        dset.attrs['Description'] = "Position of the body in the absolute frame"

        # Modes Force
        self.write_mode_force(writer, "/Modes")

        # Modes Motion
        self.write_mode_motion(writer, "/Modes")

        # Mesh file
        self.write_mesh(writer, "/Mesh")

        # Excitation force
        self.write_excitation(writer, "/Excitation")

        # Wave drift coefficients
        self.write_wave_drift(writer, "/WaveDrift")

        # Hydrostatic stiffness matrix
        self.write_hydrostatic(writer, "/Hydrostatic")

        return

    def write_mode_force(self, writer, body_modes_path="/Modes"):

        dset = writer.create_dataset(body_modes_path, "/NbForceModes", data=self.nb_force_modes)
        dset.attrs['Description'] = "Number of force modes for body number %u" % self.id

        for imode, force_mode in enumerate(self.force_modes):
            mode_path = body_modes_path + "/ForceModes/Mode_%u" % imode
            writer.create_group(mode_path)
            writer.create_database(mode_path + "/Direction", data=force_mode.direction)

            if isinstance(force_mode, hydro_db.ForceMode):
                writer.create_dataset(mode_path + "/Type", data='LINEAR')

            elif isinstance(force_mode, hydro_db.MomentMode):
                writer.create_dataset(mode_path + "/Type", data='ANGULAR')
                writer.create_dataset(mode_path + "/Point", data=force_mode.point)

        return

    def write_mode_motion(self, writer, body_modes_path="/Modes"):

        dset = writer.create_dataset(body_modes_path + "/NbMotionsModes", data=self.nb_dof)
        dset.attrs['Description'] = "Number of motion modes for body number %u" % self.id

        for idof, motion_mode in enumerate(self.motion_modes):
            mode_path = "/MotionModes/Mode_%u" % idof
            writer.create_group(mode_path)
            writer.create_dataset(mode_path + "/Direction", data=motion_mode.direction)

            if isinstance(motion_mode, hydro_db.TranslationMode):
                writer.create_dataset(mode_path + "/Type", data="LINEAR")

            elif isinstance(motion_mode, hydro_db.RotationMode):
                writer.create_dataset(mode_path + "/Type", data="ANGULAR")
                writer.create_dataser(mode_path + "/Point", data=motion_mode.point)

        return

    def write_mesh(self, writer, mesh_path="/Mesh"):

        writer.create_dataset(mesh_path + "/NbVertices", data=self.nb_vertices)
        writer.create_dataset(mesh_path + "/Vertices", data=self.vertices)
        writer.create_dataset(mesh_path + "/NbFaces", data=self.nb_faces)
        writer.create_dataset(mesh_path + "/Faces", data=self.faces)

        return

    def write_excitation(self, writer, excitation_path="/Excitation"):

        # Froude-Krylov excitation

        fk_path = excitation_path + "/FroudeKrylov"
        writer.create_group(fk_path)

        for idir, wave_dir in enumerate(self._wave_dirs):

            wave_dir_path = fk_path + "/Angle_%u" % idir
            writer.create_group(wave_dir_path)

            dset = writer.create_dataset(wave_dir_path + "/Angle", data=wave_dir)
            dset.attrs['Unit'] = 'deg'
            dset.attrs['Description'] = "Wave direction angle of the data"

            dset = writer.create_dataset(wave_dir_path + "/RealCoeffs", data=self.froude_krylov.data[:, :, idir].real)
            dset.attrs['Unit'] = ''
            dset.attrs['Description'] = "Real part of the Froude-Krylov hydrodynamic coefficients for %u forces " \
                                        "on body %u as a function of frequency" % \
                                        (self.nb_force_modes, self.id)

            dset = writer.create_dataset(wave_dir_path + "/ImagCoeffs", data=self.froude_krylov.data[:, :, idir].imag)
            dset.attrs['Unit'] = ''
            dset.attrs['Description'] = "Imaginary part of the Froude-Krylov hydrodynamic coefficients for %u " \
                                        "forces on body %u as a function of frequency" % (
                                        self.nb_force_modes, self.id)

        # Diffraction excitation

        diffraction_path = excitation_path + "/Diffraction"
        writer.create_group(diffraction_path)

        for idir, wave_dir in enumerate(self._wave_dirs):

            wave_dir_path = diffraction_path + "/Angle_%u" % idir
            writer.create_group(wave_dir_path)

            writer.create_dataset(wave_dir_path + "/Angle", data=wave_dir)
            dset.attrs['Unit'] = 'deg'
            dset.attrs['Description'] = "Wave direction angle of the data"

            writer.create_dataset(wave_dir_path + "/RealCoeffs", data=self.diffraction.data[:, :, idir].real)
            dset.attrs['Unit'] = ''
            dset.attrs['Description'] = "Real part of the diffraction hydrodynamic coefficients for %u forces " \
                                        "on body %u as a function of frequency" % \
                                        (self.nb_force_modes, self.id)

            writer.create_dataset(wave_dir_path + "/ImagCoeffs", data=self.diffraction.data[:, :, idir].imag)
            dset.attrs['Unit'] = ''
            dset.attrs['Description'] = "Imaginary part of the diffraction hydrodynamic coefficients for %u forces " \
                                        "on body %u as a function of frequency" % \
                                        (self.nb_force_modes, self.id)

        return

    def write_radiation(self, writer, radiation_path="/Radiation"):

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

            irf_path = radiation_body_motion_path + "/ImpulseResponseFunction"
            dg = writer.create_group(irf_path)
            dg.attrs['Description'] = "Impulse response function for velocity of body %u that radiates waves " \
                                      "and generates forces on body %u" % (j, self.id)

            irf_ku_path = radiation_body_motion_path + "/ImpulseResponseFunctionKu"
            dg = writer.create_group(irf_ku_path)
            dg.attrs['Description'] = "Impulse response function Ku for velocity of body %u that radiates waves " \
                                      "and generates forces on body %u" % (j, self.id)

            added_mass = self.added_mass(j)
            radiation_damping = self.radiation_damping(j)
            irf = self.irf_k(j)
            irf_ku = self.irf_ku(j)

            for imode in range(irf.size[1]):

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

        dg = writer.create_group(hydrostatic_path)

        dset = dg.create_dataset(hydrostatic_path + "/StiffnessMatrix", data=self.hydrostatic.matrix)
        dset.attrs['Description'] = "Hydrostatic stiffness matrix at the Center of gravity of the body"

        return

    def write_wave_drift(self, writer, wave_drift_path="/WaveDrift"):

        dg = writer.create_group(wave_drift_path)

        for mode in self.wave_drift.modes:
            grp_modes = dg.require_group(mode.name)

            for i in range(mode.heading.size):
                grp_dir = grp_modes.require_group("heading_%i" % mode.heading_index[i])

                # Set heading angle
                dset = grp_dir.create_dataset("heading", data=mode.heading[i])
                dset.attrs['Unit'] = 'rad'
                dset.attrs['Description'] = "Heading angle"

                # Set data
                dset = grp_dir.create_dataset("data", data=mode.data[i])
                dset.attrs['Description'] = "Wave Drift force coefficient"

        # Set frequency
        dset = dg.create_dataset("freq", data=self.wave_drift.discrete_frequency)
        dset.attrs['Unit'] = "rads"
        dset.attrs['Description'] = "Time discretization of the data"

        # Set sym
        dset = dg.create_dataset("sym_x", data=self.wave_drift.symx)
        dset.attrs['Description'] = "Symmetry along x"
        dset = dg.create_dataset('sym_y', data=self.wave_drift.symy)
        dset.attrs['Description'] = "Symmetry along y"


