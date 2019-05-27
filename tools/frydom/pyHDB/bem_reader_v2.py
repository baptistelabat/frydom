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

from math import pi,sqrt
import numpy as np
import os
from meshmagick.mmio import load_MAR
from meshmagick.mesh import Mesh, Plane

from wave_dispersion_relation_v2 import solve_wave_dispersion_relation
from pyHDB import inf
from body_db_v2 import *

class _BEMReader(object):
    """
        Class _BEMReader for checking some quantities into *.cal files.
    """

    def __init__(self, nb_face_by_wave_length=None):

        """
        Constructor of the class _BEMReader.

        Parameters
        ----------
        nb_faces_by_wavelength: float, optional
            Number of panels per wave length.
        """

        if nb_face_by_wave_length is None:
            self.nb_face_by_wave_length = 10
        else:
            self.nb_face_by_wave_length = int(nb_face_by_wave_length)

    def get_max_edge_length(self, omega, depth, grav):
        """This subroutine gets the maximum mesh's edge length allowed for accurate BEM computations for a given frequency.

        Parameters
        ----------
        omega : float
            Frequency (rad/s) at which we want to get the maximum edge length.
        depth : float
            Water depth.
        grav : float
            Gravity constant.

        Returns
        -------
        float
            Maximum edge length (meters).
        """

        wave_number = solve_wave_dispersion_relation(omega, depth, grav)
        wave_length = 2. * pi / wave_number

        return wave_length / self.nb_face_by_wave_length

    def get_max_frequency(self, edge_length, depth, grav):
        """This subroutine gets the maximum wave frequency allowed for a given mesh's edge length for accurate BEM computations

        Parameters
        ----------
        edge_length : float
            Mesh characteristic edge length
        edge_length : int, optional
            Maximum edge length.
        depth : float
            Water depth.
        grav : float
            Gravity constant.

        Returns
        -------
        float
            Maximum wave frequency (rad/s).
        """

        wave_length = self.nb_face_by_wave_length * edge_length
        wave_number = 2. * pi / wave_length
        w2 = grav * wave_number
        if depth != inf:
            w2 *= tanh(wave_number * depth)

        return sqrt(w2)

    def plot_edge_length_vs_frequency(self, depth, grav):
        """This subroutine plots the maximum allowed edge length vs the frequency

        Parameters
        ----------
        depth : float
            Water depth.
        grav : float
            Gravity constant.
        """

        # wave_number = solve_wave_dispersion_relation(self.hydro_db.omega, depth, grav) # FIXME : omega a ajoute ici.
        wave_length = 2. * pi / wave_number
        edge_length = wave_length / self.nb_face_by_wave_length

        # plt.plot(self.hydro_db.omega, edge_length) # FIXME : omega a ajoute ici.
        plt.xlabel(r'$\omega\;(rad/s)$)', fontsize=18)
        plt.ylabel(r'Edge length $(m)$', fontsize=18)
        plt.title('Edge length VS. angular frequency for a depth=%.3f m' % depth)
        plt.show()

    def test_mesh_frequency_consistency(self, pyHDB, n=10):
        """Tests whether the frequency range taken for BEM computations is consistent with the meshes discretizations.

        Parameters
        ----------
        pyHDB : object
            pyHDB object for storing the hydrodynamic database.
        n : int, optional
            Minimal number of faces by wave length. Default is 10.

        Warnings
        --------
        Returns a warning if the maximum frequency taken for BEM computations if higher than the maximum allowed
        frequency by the discretization of the meshes. It provides advices on the range of frequency that should be
        confidently used for further analysis and on the mesh discretization we should have for the given max
        frequency. This is based on the assumption that we should at least have n=10 faces by wave length.

        Note
        ----
        Reference face size is taken as the shortest edge over every mesh given (one by hydrodynamic body).
        """

        n = self.nb_face_by_wave_length

        max_edge_length_lst = []
        for body in pyHDB.bodies:
            max_edge_length_lst.append(body.mesh.max_edge_length)

        max_edge_from_mesh = max(max_edge_length_lst)
        max_edge_from_frequency = self.get_max_edge_length(pyHDB.max_wave_freq, pyHDB.depth, pyHDB.grav)
        wmax_mesh = self.get_max_frequency(max_edge_from_mesh, pyHDB.depth, pyHDB.grav)

        if max_edge_from_mesh > max_edge_from_frequency:
            wmax_bem = pyHDB.max_wave_freq
            msg = """
            \nWARNING:
            The maximum wave frequency (%.3f rad/s) chosen for BEM computations impose a maximum mesh's
            edge length of %.3f m, based on the rule that we must have at least %u faces by wave length
            (the lowest wave length corresponding to the highest wave frequency by the dispersion relation).
            Current maximum edge length is %.3f m, which is bigger.

            Any BEM data calculated at frequencies higher that %.3f rad/s may be very inaccurate and
            should not be used for analysis. Alternatively, you could provide a new refined mesh for BEM
            computations with a new maximum edge length not bigger than %.3f m which complies with the
            current frequency range.""" \
                  % (wmax_bem, max_edge_from_frequency, n, max_edge_from_mesh, wmax_mesh, max_edge_from_frequency)

            warn(msg.upper())
        else:
            msg = """
            \n-> Mesh discretization is sufficient for the given frequency range for accurate BEM computations.
            For your information, the maximum allowed frequency was %.3f rad/s. Your max frequency is %.3f rad/s.
            This is based on the assumption of having at least %u faces by wave length corresponding to the higher
            wave frequency.
            """ % (wmax_mesh, pyHDB.max_wave_freq, n)
            print msg.upper()


class NemohReaderError(Exception):
    pass


class NemohReader(_BEMReader):
    """
        Class NemohReader, derived from _BEMReader, for reading the Nemoh calculation files (*.cal).
    """

    def __init__(self, pyHDB, cal_file=None, test=True, nb_face_by_wave_length=None):  # TODO: ajouter le critere
        """ Constructor of the class NemohReader.

         Parameters
        ----------
        pyHDB : object
            pyHDB object for storing the hydrodynamic database.
        cal_file : string, optional
            Path to the Nemoh's calculation file *.cal.
        test : bool, optional
            If true (default), tests the consistency between frequency range and the mesh discretization.
        nb_faces_by_wave_length : float, optional
            Number of panels per wave length.
        """

        # Initialization of the superclass (_BEMReader) by using the super function.
        super(NemohReader, self).__init__(nb_face_by_wave_length=nb_face_by_wave_length)

        self.dirname = ''

        self.has_pressure = False

        if cal_file is not None:
            self.set_calculation_file(pyHDB, cal_file, test=test)

    def set_calculation_file(self, pyHDB, cal_file, test=True):
        """Reads the *.cal file parameters and loads the hdb and the Kochin numerical results.

        Parameters
        ----------
        pyHDB : object
            pyHDB object for storing the hydrodynamic database.
        cal_file : string
            Path to the *.cal file.
        test : bool, optional
            If true (default), tests the consistency between frequency range and the mesh discretization.

        Returns
        -------
        hydro_db : HydroDB
            The hydrodynamic database.
        """

        # Reading nemoh parameters.
        self._pre_read_nemoh_parameters(pyHDB, cal_file)
        self._read_nemoh_parameters(pyHDB, cal_file, test=test)

        # Loading hydrodynamic coefficients from Forces.dat.
        self._read_nemoh_results(pyHDB)

        # Loading Kochin function data.
        if pyHDB.has_kochin:
            self._read_kochin_function_file(pyHDB)

        return

    def _pre_read_nemoh_parameters(self,pyHDB, cal_file):

        """Reads the main Nemoh parameters from the Nemoh.cal file

        Parameters
        ----------
        pyHDB : object
            pyHDB object for storing the hydrodynamic database.
        cal_file : str
            Path (relative or absolute) to the Nemoh.cal file
        """


        if not os.path.exists(cal_file):
            raise IOError("File %s not found" % cal_file)

        with open(cal_file, 'r') as f:

            # Getting root directory of the Nemoh computation project
            abspath = os.path.abspath(cal_file)
            dirname = os.path.dirname(abspath)

            self.dirname = dirname

            def skip_line():
                f.readline()

            # Environment parameters
            skip_line()
            rho_water = float(f.readline().split()[0])
            grav = float(f.readline().split()[0])
            depth = float(f.readline().split()[0])
            if depth == 0.:
                depth = inf
            xeff, yeff = map(float, f.readline().split()[:2])

            # Number of bodies
            skip_line()
            nb_bodies = int(f.readline().split()[0])

            pyHDB.rho_water = rho_water
            pyHDB.grav = grav
            pyHDB.depth = depth
            pyHDB.x_wave_measure = xeff
            pyHDB.y_wave_measure = yeff
            pyHDB.nb_bodies = nb_bodies

            for ibody in xrange(nb_bodies):
                skip_line()
                skip_line()
                skip_line()

                # RADIATION.
                nb_dof = int(f.readline().split()[0])
                for idof in xrange(nb_dof):
                    skip_line()

                # INTEGRATION.
                nb_force = int(f.readline().split()[0])
                for iforce in xrange(nb_force):
                    skip_line()

                # Skipping additional lines
                nb_add_line = int(f.readline().split()[0])
                for i_add_line in xrange(nb_add_line):
                    skip_line()

            # Reading discretization
            skip_line()

            # Getting frequency discretization
            data = f.readline().split()[:3]
            nb_frequencies = int(data[0])
            min_frequency, max_frequency = map(float, data[1:])

            pyHDB.nb_wave_freq = nb_frequencies
            pyHDB.min_wave_freq = min_frequency
            pyHDB.max_wave_freq = max_frequency
            pyHDB.set_wave_frequencies()

            # Getting wave directions discretization
            data = f.readline().split()[:3]
            nb_wave_directions = int(data[0])
            min_wave_dir, max_wave_dir = map(float, data[1:])

            pyHDB.nb_wave_dir = nb_wave_directions
            pyHDB.min_wave_dir = min_wave_dir
            pyHDB.max_wave_dir = max_wave_dir
            pyHDB.set_wave_directions()

    def _read_nemoh_parameters(self, pyHDB, cal_file, test=True):
        """Reads the Nemoh parameters from the Nemoh.cal file

        Parameters
        ----------
        pyHDB : object
            pyHDB object for storing the hydrodynamic database.
        cal_file : str
            Path (relative or absolute) to the Nemoh.cal file
        test : bool, optional
            If True (default), it will test against consistency between frequencies taken for BEM computations and
            meshe's discretization
        """

        if not os.path.exists(cal_file):
            raise IOError("File %s not found" % cal_file)

        with open(cal_file, 'r') as f:

            # Getting root directory of the Nemoh computation project
            abspath = os.path.abspath(cal_file)
            dirname = os.path.dirname(abspath)

            self.dirname = dirname

            def skip_line():
                f.readline()

            # Environment parameters
            skip_line()
            skip_line() # Rho_water.
            skip_line() # Grav.
            skip_line() # Depth
            skip_line() # xreff and yreff.

            # Number of bodies
            skip_line()
            skip_line() # nb_bodies.

            for ibody in xrange(pyHDB.nb_bodies):
                skip_line()
                meshfile = str(f.readline().split()[0])
                abs_meshfile = os.path.join(dirname, meshfile)

                # Instantiating mesh object
                vertices, faces = load_MAR(abs_meshfile)
                mesh = Mesh(vertices, faces)

                nb_vertices, nb_faces = map(int, f.readline().split()[:2])
                # Verification of mesh information consistency
                assert nb_vertices == mesh.nb_vertices
                assert nb_faces == mesh.nb_faces

                # De-symmetrizing meshes
                with open(abs_meshfile, 'r') as fmesh:
                    _, isym = fmesh.readline().split()
                if int(isym) == 1:  # Mesh is symmetric
                    mesh.symmetrize(Plane([0, 1, 0]))

                mesh.name, _ = os.path.splitext(meshfile)

                # Instantiating BodyDB.
                body = BodyDB(ibody, pyHDB.nb_bodies, pyHDB.nb_wave_freq, pyHDB.nb_wave_dir , mesh)

                # RADIATION.
                nb_dof = int(f.readline().split()[0])
                for idof in xrange(nb_dof):
                    dof = f.readline().split()[:7]
                    motion_type = int(dof[0])
                    direction = map(float, dof[1:4])
                    point = map(float, dof[4:])

                    if motion_type == 1:  # Translation
                        if(direction[0] == 1):
                            body.Motion_mask[0] = 1
                        elif(direction[1] == 1):
                            body.Motion_mask[1] = 1
                        elif(direction[2] == 1):
                            body.Motion_mask[2] = 1
                        else:
                            "The linear motion direction must be a unit vector ex, ey or ez. Generalized modes are not taken into account."
                            exit()
                    elif motion_type == 2:  # Rotation
                        if (direction[0] == 1):
                            body.Motion_mask[3] = 1
                        elif (direction[1] == 1):
                            body.Motion_mask[4] = 1
                        elif (direction[2] == 1):
                            body.Motion_mask[5] = 1
                        else:
                            "The angular motion direction must be a unit vector ex, ey or ez. Generalized modes are not taken into account."
                            exit()
                    else:
                        raise UnknownMotionMode(motion_type, abspath)

                # INTEGRATION.
                nb_force = int(f.readline().split()[0])
                for iforce in xrange(nb_force):
                    force = f.readline().split()[:7]
                    force_type = int(force[0])
                    direction = map(float, force[1:4])
                    point = map(float, force[4:])
                    if force_type == 1:  # Pure force
                        if (direction[0] == 1):
                            body.Force_mask[0] = 1
                        elif (direction[1] == 1):
                            body.Force_mask[1] = 1
                        elif (direction[2] == 1):
                            body.Force_mask[2] = 1
                        else:
                            "The force direction must be a unit vector ex, ey or ez. Generalized modes are not taken into account."
                            exit()
                    elif force_type == 2:
                        if (direction[0] == 1):
                            body.Force_mask[3] = 1
                            body.point[0,:] = point
                        elif (direction[1] == 1):
                            body.Force_mask[4] = 1
                            body.point[1, :] = point
                        elif (direction[2] == 1):
                            body.Force_mask[5] = 1
                            body.point[2, :] = point
                        else:
                            "The angular motion direction must be a unit vector ex, ey or ez. Generalized modes are not taken into account."
                            exit()
                    else:
                        raise UnknownForceMode(force_type, abspath)

                # Skipping additional lines.
                nb_add_line = int(f.readline().split()[0])
                for i_add_line in xrange(nb_add_line):
                    skip_line()

                pyHDB.append(body)

            # Reading discretization.
            skip_line()

            # Getting frequency discretization.
            skip_line()

            # Getting wave directions discretization.
            skip_line()

            # Getting post-processing information
            skip_line()

            # Impulse response function
            data = f.readline().split()[:3]
            # has_irf = bool(int(data[0]))
            # if has_irf:
            #     self.dt_irf, self.tf_irf = map(float, data[1:])

            # TODO: si ok lire  -> remettre en place si besoin...
            # Show pressure
            skip_line()
            # self.has_pressure = bool(int(f.readline().split()[0])) # Enlever le skip_line() ci-dessus si décommenté.

            # Kochin functions.
            data = f.readline().split()[:3]
            pyHDB.has_kochin = bool(float(data[0]))
            if pyHDB.has_kochin:
                pyHDB.nb_angle_kochin = int(data[0])
                pyHDB.min_angle_kochin, pyHDB.max_angle_kochin = map(float, data[1:])
                pyHDB.set_wave_directions_Kochin()


            # # TODO: si ok, lire  -> remettre en place si besoin...
            # # Free surface elevation
            # data = f.readline().split()[:4]
            # has_fs_elevation = bool(int(data[0]))
            # if has_fs_elevation:
            #     self.nx_fs, self.ny_fs = map(int, data[:2])
            #     self.xlim_fs, self.ylim_fs = map(float, data[2:])

        if test:
            self.test_mesh_frequency_consistency(pyHDB)

    def _read_nemoh_results(self,pyHDB):  # FIXME: methode de reader nemoh
        """Reads the Nemoh's results from the Forces.dat file.

        Parameter
        ---------
        pyHDB : object
            pyHDB object for storing the hydrodynamic database.
        """

        ############################################################################
        #                                Reading
        ############################################################################

        bem_results_file = os.path.join(self.dirname, 'results', 'Forces.dat')

        if not os.path.exists(bem_results_file):
            raise NemohReaderError("Nemoh result file 'Forces.dat' does not exists in directory %s.\n\n"
                                   ">>> PLEASE VERIFY THAT NEMOH COMPUTATIONS HAVE BEEN RUN.\n" %
                                   os.path.join(self.dirname, 'results'))

        # Raw data.
        with open(bem_results_file, 'r') as f:
            data = np.asarray(f.read().split(), dtype=np.float)

        nw = pyHDB.nb_wave_freq
        nbeta = pyHDB.nb_wave_dir
        nbodies = pyHDB.nb_bodies
        nb_forces = pyHDB.nb_forces
        nb_dof = pyHDB.nb_motions

        # Reshape.
        res = data.reshape((nb_forces, nw, 2 * (nbeta + nb_dof)))

        # Physical structures.
        diffraction_data = res[:, :, :2 * nbeta:2] * np.exp(1j * res[:, :, 1:2 * nbeta:2])
        added_mass_data = res[:, :, 2 * nbeta::2]
        radiation_damping_data = res[:, :, 2 * nbeta + 1::2]

        # (nb_forces, nw, nb_dof) -> (nb_forces, nb_dof, nw) for added mass and damping matrices.
        added_mass_data_v2 = np.zeros((nb_forces, nb_dof, nw), dtype = np.float)
        radiation_damping_data_v2 = np.zeros((nb_forces, nb_dof, nw), dtype = np.float)
        for iw in range(0, nw):
            added_mass_data_v2[:, :, iw] = added_mass_data[:, iw, :]
            radiation_damping_data_v2[:, :, iw] = radiation_damping_data[:, iw, :]

        ############################################################################
        #                              Dispatching
        ############################################################################

        # Diffraction.
        for body in pyHDB.bodies:
            i_force = 0
            for j in range(0,6):
                if(body.Force_mask[j] == 1): # Force activated.
                    body.Diffraction[j,:,:] = diffraction_data[i_force,:,:]
                    i_force = i_force + 1

        # Added mass and damping.
        for body_p in pyHDB.bodies:
            i_force = 0
            for p in range(0,6):
                if(body_p.Force_mask[p] == 1): # Force activated.
                    for body_q in pyHDB.bodies:
                        i_motion = 0
                        for q in range(0,6):
                            if(body_q.Motion_mask[q] == 1): # Motion activated.
                                body_p.Added_mass[p, q, :] = added_mass_data_v2[i_force, i_motion,:]
                                body_p.Damping[p, q, :] = radiation_damping_data_v2[i_force, i_motion, :]
                                i_motion = i_motion + 1
                    i_force = i_force + 1

    def _read_kochin_function_file(self,pyHDB):
        """Read the Kochin function values data from Nemoh's Kochin.%5u.dat files.

        Parameter
        ---------
        pyHDB : object
            pyHDB object for storing the hydrodynamic database.
        """

        res_dir = os.path.join(self.dirname, 'results')

        def read(i):
            filename = os.path.join(res_dir, 'Kochin.%5u.dat' % i)
            with open(filename, 'r') as f:
                data = f.read()
            arr = np.array(data.split(), dtype=np.float).reshape((-1, 3))
            return arr[:, 1] * np.exp(1j * arr[:, 2])

        def read_wave_dir(i):
            # Read the wave direction discretization of a Kochin elementary file.
            filename = os.path.join(res_dir, 'Kochin.%5u.dat' % i)
            with open(filename, 'r') as f:
                data = f.read()
            arr = np.array(data.split(), dtype=np.float).reshape((-1, 3))
            return arr[:, 0]

        ntheta = pyHDB.nb_angle_kochin
        nw = pyHDB.nb_wave_freq
        nbeta = pyHDB.nb_wave_dir
        nbodies = pyHDB.nb_bodies
        pyHDB.kochin_diffraction = np.zeros((nbeta, nw, ntheta), dtype=np.complex)
        pyHDB.kochin_radiation = np.zeros((6*nbodies, nw, ntheta), dtype=np.complex)

        # Real and imaginary parts of the Kochin functions for every elementary problem.
        i_bem_problem = 0
        for ifreq in xrange(nw):

            # Diffraction problems.
            for iwave in xrange(nbeta):
                i_bem_problem += 1
                pyHDB.kochin_diffraction[iwave, ifreq, :] = read(i_bem_problem)*np.exp(1j * pi/2.) # np.exp(1j * pi/2.) due to the Nemoh convention.

            # Radiation problems.
            for body in pyHDB.bodies:
                for imotion in range(0,6):
                    if(body.Motion_mask[imotion] == 1):
                        i_bem_problem += 1
                        pyHDB.kochin_radiation[imotion, ifreq, :] = read(i_bem_problem)*np.exp(1j * pi/2.) # np.exp(1j * pi/2.) due to the Nemoh convention.

        # Wave direction discretization of the Kochin functions.
        pyHDB.angle_kochin = read_wave_dir(1)  # The discretization is the same for every elementary problem, so the first file is used by default.
