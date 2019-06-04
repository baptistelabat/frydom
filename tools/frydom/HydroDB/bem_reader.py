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

import numpy as np
import os
from math import pi, sqrt, tanh
from warnings import warn
import matplotlib.pyplot as plt
import cPickle

from wave_dispersion_relation import solve_wave_dispersion_relation
from hydro_db import (inf, HydroDB, HydroBody, TranslationMode, RotationMode,
                      ForceMode, MomentMode, UnknownForceMode, UnknownMotionMode)

from meshmagick.mmio import load_MAR  # FIXME: Mesh doit etre builtin dans frydom et plane dans geometry,
from meshmagick.mesh import Mesh, Plane

# FIXME: les classes BEMReader ne se justifient pas, il faut juste avoir une fonction qui lit et renvoie un objet
# HydroDB...


class _BEMReader(object):  # TODO: le parametre n pour le nb de facette par lambda devra etre reglable...

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

        self.hydro_db = HydroDB()
        
        if nb_face_by_wave_length is None:
            self.nb_face_by_wave_length = 10
        else:
            self.nb_face_by_wave_length = int(nb_face_by_wave_length)
    
    def get_max_edge_length(self, omega):  # FIXME: methode generique BEM
        """This subroutine gets the maximum mesh's edge length allowed for accurate BEM computations for a given frequency.

        Parameters
        ----------
        omega : float
            Frequency (rad/s) at which we want to get the maximum edge length.
        n : int, optional
            Minimal number of faces by wave length. Default is 10.

        Returns
        -------
        float
            Maximum edge length (meters).
        """
        wave_number = solve_wave_dispersion_relation(omega, self.hydro_db.depth, self.hydro_db.grav)
        wave_length = 2. * pi / wave_number
        return wave_length / self.nb_face_by_wave_length
    
    def get_max_frequency(self, edge_length):
        """This subroutine gets the maximum wave frequency allowed for a given mesh's edge length for accurate BEM computations

        Parameters
        ----------
        edge_length : float
            Mesh characteristic edge length
        n : int, optional
            Minimal number of faces by wave length. Default is 10.

        Returns
        -------
        float
            Maximum wave frequency (rad/s).
        """
        wave_length = self.nb_face_by_wave_length * edge_length
        wave_number = 2. * pi / wave_length
        w2 = self.hydro_db.grav * wave_number
        if self.hydro_db.depth != inf:
            w2 *= tanh(wave_number * self.hydro_db.depth)
        return sqrt(w2)
    
    def plot_edge_length_vs_frequency(self):
        """This subroutine plots the maximum allowed edge length vs the frequency

        Parameters
        ----------
        n : int, optional
            Minimal number of faces by wave length. Default is 10.
        """

        wave_number = solve_wave_dispersion_relation(self.hydro_db.omega, self.hydro_db.depth, self.hydro_db.grav)
        wave_length = 2. * pi / wave_number
        edge_length = wave_length / self.nb_face_by_wave_length
        
        plt.plot(self.hydro_db.omega, edge_length)
        plt.xlabel(r'$\omega\;(rad/s)$)', fontsize=18)
        plt.ylabel(r'Edge length $(m)$', fontsize=18)
        plt.title('Edge length VS. angular frequency for a depth=%.3f m' % self.hydro_db.depth)
        plt.show()
    
    def test_mesh_frequency_consistency(self, n=10):  # FIXME: methode generique BEM
        """Tests whether the frequency range taken for BEM computations is consistent with the meshes discretizations.

        Parameters
        ----------
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
        for body in self.hydro_db.body_mapper.bodies:
            max_edge_length_lst.append(body.mesh.max_edge_length)
            
        max_edge_from_mesh = max(max_edge_length_lst)
        max_edge_from_frequency = self.get_max_edge_length(self.hydro_db.max_frequency)
        wmax_mesh = self.get_max_frequency(max_edge_from_mesh)
        
        if max_edge_from_mesh > max_edge_from_frequency:
            wmax_bem = self.hydro_db.max_frequency
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
            """ % (wmax_mesh, self.hydro_db.max_frequency, n)
            print msg.upper()


class NemohReaderError(Exception):
    pass


class NemohReader(_BEMReader):

    """
        Class NemohReader, derived from _BEMReader, for reading the Nemoh calculation files (*.cal).
    """

    def __init__(self, cal_file=None, test=True, nb_face_by_wave_length=None):  # TODO: ajouter le critere
        """ Constructor of the class NemohReader.

         Parameters
        ----------
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
            self.set_calculation_file(cal_file, test=test)
    
    def set_calculation_file(self, cal_file, test=True):
        """Reads the *.cal file parameters and loads the hdb and the Kochin numerical results.

        Parameters
        ----------
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
        self._read_nemoh_parameters(cal_file, test=test)
        
        # Loading hydrodynamic coefficients from Forces.dat.
        self._read_nemoh_results()
        
        # Loading Kochin function data.
        # if self.hydro_db.has_kochin:
        #     self._read_kochin_function_file()
        
        return self.hydro_db
    
    def _read_nemoh_parameters(self, cal_file, test=True):
        """Reads the Nemoh parameters from the Nemoh.cal file

        Parameters
        ----------
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
            rho_water = float(f.readline().split()[0])
            grav = float(f.readline().split()[0])
            depth = float(f.readline().split()[0])
            if depth == 0.:
                depth = inf
            xeff, yeff = map(float, f.readline().split()[:2])
            
            # Number of bodies
            skip_line()
            nb_bodies = int(f.readline().split()[0])
            
            self.hydro_db.rho_water = rho_water
            self.hydro_db.grav = grav
            self.hydro_db.depth = depth
            self.hydro_db.x_wave_measure = xeff
            self.hydro_db.y_wave_measure = yeff
            
            body_mapper = self.hydro_db.body_mapper
            
            motion_mod_idx = force_mod_idx = 0
            # bodies = []
            for ibody in xrange(nb_bodies):
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
                
                # Instantiating HydroBody
                body = HydroBody(ibody, mesh)
                
                # RADIATION
                # radiation_modes = []
                nb_dof = int(f.readline().split()[0])
                for idof in xrange(nb_dof):
                    dof = f.readline().split()[:7]
                    motion_type = int(dof[0])
                    direction = map(float, dof[1:4])
                    point = map(float, dof[4:])
                    
                    if motion_type == 1:  # Translation
                        motion_mod = TranslationMode(direction, point, motion_mod_idx, ibody)
                    elif motion_type == 2:  # Rotation
                        motion_mod = RotationMode(direction, point, motion_mod_idx, ibody)
                    else:
                        raise UnknownMotionMode(motion_type, abspath)
                    body_mapper._motion_mod_dict[motion_mod_idx] = (ibody, idof)  # TODO: voir a faire ca mieux...
                    motion_mod_idx += 1
                    
                    body.append_motion_mode(motion_mod)
                
                # INTEGRATION
                # force_modes = []
                nb_force = int(f.readline().split()[0])
                for iforce in xrange(nb_force):
                    force = f.readline().split()[:7]
                    force_type = int(force[0])
                    direction = map(float, force[1:4])
                    point = map(float, force[4:])
                    if force_type == 1:  # Pure force
                        force_mod = ForceMode(direction, point, force_mod_idx, ibody)
                    elif force_type == 2:
                        force_mod = MomentMode(direction, point, force_mod_idx, ibody)
                    else:
                        raise UnknownForceMode(force_type, abspath)
                    body_mapper._force_mod_dict[force_mod_idx] = (ibody, iforce)  # TODO: voir a faire ca mieux
                    force_mod_idx += 1
                    
                    body.append_force_mode(force_mod)
                
                # Skipping additional lines
                nb_add_line = int(f.readline().split()[0])
                for i_add_line in xrange(nb_add_line):
                    skip_line()
                
                body_mapper.append(body)
            
            # Reading discretization
            skip_line()
            
            # Getting frequency discretization
            data = f.readline().split()[:3]
            nb_frequencies = int(data[0])
            min_frequency, max_frequency = map(float, data[1:])
            
            self.hydro_db.set_frequencies(min_frequency, max_frequency, nb_frequencies)
            
            # Getting wave directions discretization
            data = f.readline().split()[:3]
            
            nb_wave_directions = int(data[0])
            min_wave_dir, max_wave_dir = map(float, data[1:])
            
            self.hydro_db.set_wave_direction(min_wave_dir, max_wave_dir, nb_wave_directions)
            
            # Getting post-processing information
            skip_line()
            
            # Impulse response function
            data = f.readline().split()[:3]
            # has_irf = bool(int(data[0]))
            # if has_irf:
            #     self.dt_irf, self.tf_irf = map(float, data[1:])
            
            # TODO: si ok lire  -> remettre en place si besoin...
            # Show pressure
            self.has_pressure = bool(int(f.readline().split()[0]))
            
            # Kochin function
            data = f.readline().split()[:3]
            self.hydro_db.has_kochin = bool(float(data[0]))
            if self.hydro_db.has_kochin:
                self.hydro_db.nb_dir_kochin = int(data[0])
                self.hydro_db.min_dir_kochin, self.hydro_db.max_dir_kochin = map(float, data[1:])
                
            # # TODO: si ok, lire  -> remettre en place si besoin...
            # # Free surface elevation
            # data = f.readline().split()[:4]
            # has_fs_elevation = bool(int(data[0]))
            # if has_fs_elevation:
            #     self.nx_fs, self.ny_fs = map(int, data[:2])
            #     self.xlim_fs, self.ylim_fs = map(float, data[2:])
        
        if test:
            self.test_mesh_frequency_consistency()
    
    def _read_nemoh_results(self):  # FIXME: methode de reader nemoh
        """Reads the Nemoh's results from the Forces.dat file"""
        bem_results_file = os.path.join(self.dirname, 'results', 'Forces.dat')
        
        if not os.path.exists(bem_results_file):
            raise NemohReaderError("Nemoh result file 'Forces.dat' does not exists in directory %s.\n\n"
                                   ">>> PLEASE VERIFY THAT NEMOH COMPUTATIONS HAVE BEEN RUN.\n" %
                                   os.path.join(self.dirname, 'results'))
        
        with open(bem_results_file, 'r') as f:
            data = np.asarray(f.read().split(), dtype=np.float)
        
        hydro_db = self.hydro_db
        
        nw = hydro_db.nb_frequencies
        n_wave = hydro_db.diffraction_db.nb_wave_dir
        n_forces = hydro_db.body_mapper.nb_force_modes
        n_dof = hydro_db.body_mapper.nb_motion_modes
        
        res = data.reshape((n_forces, nw, 2 * (n_wave + n_dof)))
        
        diffraction_data = res[:, :, :2 * n_wave:2] * np.exp(1j * res[:, :, 1:2 * n_wave:2])
        added_mass_data = res[:, :, 2 * n_wave::2]
        radiation_damping_data = res[:, :, 2 * n_wave + 1::2]  # TODO: adapter tout ca au nouveau format RadiationDB
        
        hydro_db.set_diffraction_data(diffraction_data)  # FIXME: bug, on recupere un copie...
        hydro_db.set_radiation_data(radiation_damping_data, added_mass_data)
        
    def _read_kochin_function_file(self):  # TODO: A REMETTRE D'APLOMB APRES MIGRATION
        """Read the Kochin function values data from Nemoh's Kochin.%5u.dat files"""
        
        raise NotImplementedError
        
        res_dir = os.path.join(self.dirname, 'results')
        
        def read(i):
            filename = os.path.join(res_dir, 'Kochin.%5u.dat' % i)
            with open(filename, 'r') as f:
                data = f.read()
            arr = np.array(data.split(), dtype=np.float).reshape((-1, 3))
            return arr[:, 1] * np.exp(1j * arr[:, 2])
        
        ntheta = self.nb_dir_kochin
        nw = self.nb_frequencies
        self.kochin_diffraction = np.zeros((self.nb_wave_directions, nw, ntheta), dtype=np.complex)
        self.kochin_radiation = np.zeros((self.nb_motion_modes, nw, ntheta), dtype=np.complex)
        
        i_bem_problem = 0
        for ifreq in xrange(self.nb_frequencies):
            
            for iwave in xrange(self.nb_wave_directions):
                # Diffraction problem
                i_bem_problem += 1
                self.kochin_diffraction[iwave, ifreq, :] = read(i_bem_problem)
            
            for imotion in xrange(self.nb_motion_modes):
                # Radiation problem
                i_bem_problem += 1
                self.kochin_radiation[imotion, ifreq, :] = read(i_bem_problem)
