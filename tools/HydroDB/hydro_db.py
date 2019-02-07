#!/usr/bin/env python
#  -*- coding: utf-8 -*-
"""Module to load hydrodynamic BEM data from Nemoh code"""

import numpy as np
from math import degrees, radians, pi
from copy import deepcopy
from scipy import interpolate

import time

import matplotlib.pyplot as plt

# from meshmagick.mmio import load_MAR
# from meshmagick.mesh import Mesh, Plane


# TODO: creer un module pour la lecture des donnees Nemoh a part du module gerant les classe relatives a l'hydro du premier ordre...

# TODO: mettre les exceptions dans un module a part

inf = float('inf')  # Definition of infinity for depth


# TODO: creer une classe HydroBodyMapper qui remplace la liste bodies et qui gere le mapping entre
#  les corps et les donnees de la base de donnees hydro. A un index de corps + un index de force ou
#  de mouvement correspond respectivement un index de ligne ou de colonne dans la base de donnees
#  hydro.
class _FreqDB(object):
    def __init__(self):
        self._min_frequency = 0.
        self._max_frequency = 0.
        self._nb_frequencies = 0
        
        self._iwcut = None
    
    @property
    def min_frequency(self):
        return self._min_frequency
    
    @property
    def max_frequency(self):
        if self._iwcut is None:
            return self._max_frequency
        else:
            return self.omega[-1]
    
    @property
    def nb_frequencies(self):
        return len(self.omega)
    
    @property
    def omega(self):  # TODO: integrer le _icut
        """Frequency array of BEM computations in rad/s

        Returns
        -------
        np.ndarray
        """
        w = self.get_full_omega()
        if self._iwcut is None:
            return w
        else:
            return w[:self._iwcut]
    
    def get_full_omega(self):
        return np.linspace(self._min_frequency, self._max_frequency, self._nb_frequencies, dtype=np.float)
    
    @property
    def wcut(self):
        if self._iwcut is None:
            return None
        else:
            w = self.get_full_omega()
            return w[self._iwcut]
        
    @wcut.setter
    def wcut(self, wcut):  # TODO: finir l'implementation
        if wcut is None:
            self._iwcut = None
        else:
            assert self._min_frequency < wcut <= self._max_frequency
            w = self.get_full_omega()
            self._iwcut = np.where(w >= wcut)[0][0]  # TODO: a verifier
    
    def set_frequencies(self, wmin, wmax, nw):
        assert 0. < wmin < wmax
        assert nw > 0
        
        self._min_frequency = wmin
        self._max_frequency = wmax
        self._nb_frequencies = nw
    
    def get_frequency_discretization(self):  # FIXME: BUG tres probable avec wcut !!!
        return self._min_frequency, self._max_frequency, self._nb_frequencies
    

class HydroDB(_FreqDB):
    """class to store frequency dependent hydrodynamics data coming from BEM codes in frequency domain.
    
    Attributes
    ----------
    rho_water : float
        density of water (kg/m**3). Default is 1000 kg/m**3
    grav : float
        acceleration of the gravity field. Default is 9.81 m/s**2
    depth : float
        local water depth in m. Default is inf, meaning an infinite water depth.
    x_wave_measure : float
        x position of the wave probe used for wave elevation
    y_wave_measure : float
        y position of the wave probe used for wave elevation
    nb_wave_dir : int
        Number of wave directions used in computations
    min_wave_dir : float
        
    """
    def __init__(self):
        super(HydroDB, self).__init__()
        
        # General parameters
        self.rho_water = 1000.
        self.grav = 9.81
        self.depth = inf
        self.x_wave_measure = 0.
        self.y_wave_measure = 0.
        
        self.nb_wave_dir = 0  # TODO: mettre dans diffraction uniquement
        self.min_wave_dir = 0.
        self.max_wave_dir = 0.
        self._wave_dirs = np.array([])
        
        self.has_kochin = False  # TODO: mettre dans Kochin uniquement
        self.nb_dir_kochin = 0
        self.min_dir_kochin = 0.
        self.max_dir_kochin = 0.
        
        self.body_mapper = HydroBodySetMapping()
        
        self._radiation_db = RadiationDB()  # IRF est genere a partir de radiation...
        self._diffraction_db = DiffractionDB()  # TODO: les frequences seront ajoutees lors d'une requete sur la ppte !

        
        self._has_froude_krylov = False
        self._froude_krylov_db = FroudeKrylovDB()
        
    @property
    def nb_bem_problems(self):  # TODO: methode de HydroDB
        """Total number of problems solved during the BEM computations

        It is nw * (ntheta + nradiation)

        Returns
        -------
        int
        """
        nw = self.nb_frequencies
        ntheta = self.nb_wave_dir
        return nw * (ntheta + self.body_mapper.nb_motion_modes)

    @property
    def radiation_db(self):  # TODO: renseigner les frequences et wcut...
        db = deepcopy(self._radiation_db)
        db.set_frequencies(self._min_frequency, self._max_frequency, self._nb_frequencies)
        db.wcut = self.wcut
        db.body_mapper = self.body_mapper
        return db
    
    @property
    def diffraction_db(self):  # TODO: renseigner les frequences et wcut...
        db = deepcopy(self._diffraction_db)
        db.set_frequencies(self._min_frequency, self._max_frequency, self._nb_frequencies)
        db.set_wave_dir(self.min_wave_dir, self.max_wave_dir, self.nb_wave_dir)
        db.wcut = self.wcut
        db.body_mapper = self.body_mapper
        db._x_wave_measure = self.x_wave_measure
        db._y_wave_measure = self.y_wave_measure
        return db
    
    @property
    def froude_krylov_db(self):
        db = self._froude_krylov_db
        if not self._has_froude_krylov:
            db.set_frequencies(self._min_frequency, self._max_frequency, self._nb_frequencies)
            db.set_wave_dir(self.min_wave_dir, self.max_wave_dir, self.nb_wave_dir)
            db.wcut = self.wcut
            db.body_mapper = self.body_mapper
            db.eval(self.rho_water, self.grav, self.depth, self.x_wave_measure, self.y_wave_measure)
            
            db._x_wave_measure = self.x_wave_measure
            db._y_wave_measure = self.y_wave_measure
            
            self._froude_krylov_db = db
            self._has_froude_krylov = True
            
        return db
    
    @property
    def wave_excitation_db(self):
        diffraction_db = self.diffraction_db
        froude_krylov_db = self.froude_krylov_db
        
        db = WaveExcitationDB()
        db.set_frequencies(self._min_frequency, self._max_frequency, self._nb_frequencies)
        db.set_wave_dir(self.min_wave_dir, self.max_wave_dir, self.nb_wave_dir)
        db.wcut = self.wcut
        db.body_mapper = self.body_mapper
        
        db.data = diffraction_db.data + froude_krylov_db.data  # Summing diffraction and Froude-Krylov contributions
        
        db._x_wave_measure = self.x_wave_measure
        db._y_wave_measure = self.y_wave_measure
        
        return db

    @property
    def wave_dirs(self):
        return self._wave_dirs

    def set_wave_direction(self, min_wave_dir, max_wave_dir, nb_wave_dir):
        # self._diffraction_db.set_wave_dir(min_wave_dir, max_wave_dir, nb_wave_dir)
        self.min_wave_dir = min_wave_dir
        self.max_wave_dir = max_wave_dir
        self.nb_wave_dir = nb_wave_dir
        
    def set_diffraction_data(self, data):
        # TODO: faire verif !!
        self._diffraction_db.data = data
        
    def set_radiation_data(self, radiation_damping, added_mass, infinite_added_mass=None):
        # TODO: faire verifs !!
        self._radiation_db.set_data(radiation_damping, added_mass, cm_inf=infinite_added_mass)


class WaveExcitationError(Exception):
    pass

    
class WaveExcitation(_FreqDB):
    _type = ''
    _type_extended = ''
    
    def __init__(self):
        super(WaveExcitation, self).__init__()
        # super(DiffractionDB, self).__init__()
        # self.min_frequency = 0.
        # self.max_frequency = 0.
        # self.nb_frequency = 0
        
        self._x_wave_measure = 0.
        self._y_wave_measure = 0.
        
        self.min_wave_dir = 0.
        self.max_wave_dir = 0.
        self.nb_wave_dir = 0
        
        self.data = np.empty(0, dtype=np.complex)
        
        self.body_mapper = HydroBodySetMapping()
    
    def get_wave_measure_position(self):
        return self._x_wave_measure, self._y_wave_measure
    
    def set_data(self, diffraction_data):
        # TODO: faire les verifs...
        self.data = np.asarray(diffraction_data, dtype=np.complex)
    
    def get_body_data(self, ibody):
        nb_dof = self.body_mapper.bodies[ibody].nb_dof
        
        istart = self.body_mapper.get_general_force_index(ibody, 0)
        # istop = self.body_mapper.get_general_force_index(ibody, nb_dof-1)
        
        return self.data[istart:istart+nb_dof, :, :]
    
    def interp(self, omega, wave_dir, ibody=None, unit='deg'):
        """Interpolate wave excitation hydrodynamic coefficient over frequency and wave directions
        
        Parameters
        ----------
        omega : np.ndarray
            angular frequencies at which the database has to be interpolated
        wave_dir : np.ndarray
            wave directions at which the database has to be interpolated
        ibody : int, optional
            The id of the hydrodynamic body in the database. If None (default), the full results for every bodies in
            interaction will be interpolated
        unit = 'str', optional
            The angle unit used in wave_dir. Default is 'deg'. May also be 'rad'
        
        Returns
        -------
        np.ndarray[complex]
            Complex array of hydrodynamic coefficients interpolated at requested places.
            
        Raises
        ------
        Error if the requested values are outside the ranges of the database
        
        Notes
        -----
        Interpolation in database allows to set a linear wavefield with a different discretization than the database.
        It is of practical usage when the database has been realized with numerous values so that it covers the
        broader range of simulations but are over-discretized with respect to the needs in excitation force
        computations as they would give a too computational demanding double sum.
        """
        
        if ibody is None:
            data = self.data
        else:
            data = self.get_body_data(ibody)
        
        if unit == 'deg':
            wave_dir = np.radians(wave_dir)
        elif unit != 'rad':
            raise WaveExcitationError("Angle unit must be 'deg' or 'rad', not %s" % str(unit))
        
        if np.isscalar(omega):
            omega = np.array([omega], dtype=np.float)
        
        if np.isscalar(wave_dir):
            wave_dir = np.array([wave_dir], dtype=np.float)
        
        wmin, wmax, nw = omega[0], omega[-1], len(omega)
        min_wave_dir, max_wave_dir, nb_wave_dir = wave_dir[0], wave_dir[-1], len(wave_dir)
        
        # Testing if requested values are consistent with the current database (no extrapolation allowed)
        if wmin < self.min_frequency or wmax > self.max_frequency:
            raise WaveExcitationError('Interpolation values for %s in frequency are outside the values that are '
                                      'available in the database [%.3f, %.3f] rad/s. No extrapolation are allowed on '
                                      'hydrodynamic databases.'
                                      % (type(self), self.min_frequency, self.max_frequency))
        
        if min_wave_dir < self.min_wave_dir or max_wave_dir > self.max_wave_dir:
            raise WaveExcitationError('Interpolation values for %s in wave direction are outside the values that are '
                                      'available in the database [%.3f, %.3f] deg. No extrapolation are allowed on '
                                      'hydrodynamic databases.'
                                      % (type(self), degrees(self.min_wave_dir), degrees(self.max_wave_dir)))
        
        # Initial values from the database
        omega_db = self.omega
        beta_db = self.wave_dir
        
        n_force, nwi, nbi = data.shape[0], nw, nb_wave_dir
        
        fexc_interp = np.zeros((n_force, nwi, nbi), dtype=np.complex)
        
        for iforce in xrange(data.shape[0]):
            # TODO: voir pour une interpolation collective (sans boucle)
            # TODO: voir pour une interpolation directement en complexes (pas real puis imag)
            fexc = data[iforce, :, :]  # is nw x nbeta
            
            
            # TODO: voir si on ne stocke pas de base les interpolateurs dans la database a l'initialisation des donnees
            interp_real = interpolate.interp2d(beta_db, omega_db, fexc.real)  # type: interpolate.interp2d
            interp_imag = interpolate.interp2d(beta_db, omega_db, fexc.imag)  # type: interpolate.interp2d
            
            fexc_interp[iforce, :, :] = interp_real(wave_dir, omega) + 1j * interp_imag(wave_dir, omega)
            
        return fexc_interp

    def set_wave_dir(self, min_wave_dir, max_wave_dir, nb_wave_dir):
        assert min_wave_dir <= max_wave_dir
        assert nb_wave_dir > 0
        self.min_wave_dir = min_wave_dir
        self.max_wave_dir = max_wave_dir
        self.nb_wave_dir = nb_wave_dir
    
    @property
    def wave_dir(self):  # TODO: methode de DiffractionDB (et excitation DB)
        """Wave direction angles array of BEM computations in radians

        Returns
        -------
        np.ndarray
            angles array in radians
        """
        return np.radians(np.linspace(self.min_wave_dir, self.max_wave_dir, self.nb_wave_dir, dtype=np.float))

    def get_complex_coeffs(self, ibody, iforce, iwave=0):  # TODO: Methode de DiffractionDB
        """Get the diffraction coefficients for a body's mode and a given wave direction

        Parameters
        ----------
        ibody : int
            The index of the body
        iforce : int
            The index of the wanted body's integration mode
        iwave : int, float
            Wave direction. If an int is given, it is used as an index in the wave_dir array. If a float is given, the
            closest wave direction is given

        Returns
        -------
        np.ndarray
            Complex array of the diffraction coefficients
        """
        if isinstance(iwave, float):
            # Finding the closest wave direction
            iwave = np.fabs(self.wave_dir - radians(iwave)).argmin()
        
        # gal_idx = self.body_mapper.
        gal_idx = self.body_mapper.get_general_force_index(ibody, iforce)
        if self._iwcut is None:
            return self.data[gal_idx, :, iwave]  # is
        else:
            return self.data[gal_idx, :self._iwcut, iwave]

    def plot(self, ibody, iforce, iwave=0, **kwargs):
        """Plots the diffraction response function of a given modes set

        Parameters
        ----------
        ibody : int
            The index of the body
        iforce : int
            The index of the body's force mode
        iwave : int, float
            The index of the wave direction (if an integer is given) or the value of the wave_direction in degrees in a
            float is given. In the latter case, the most close angle is taken (not necessarily the one that is given
            here...)
        kwargs: optional
            Arguments that are to be used by pyplot
        """
    
        coeffs = self.get_complex_coeffs(ibody, iforce, iwave)
        xlabel = r'$\omega (rad/s)$'
        ylabel1 = r'$|F_{%s}(\omega)|$' % self._type
        ylabel2 = r'$\angle\left[F_{%s}(\omega)\right] (deg)$' % self._type
        
        mode = self.body_mapper.bodies[ibody].force_modes[iforce]
        
        # mode = self.bodies[ibody].force_modes[iforce]
        if isinstance(mode, ForceMode):
            ylabel1 += r' $(N/m)$'
        elif isinstance(mode, MomentMode):
            ylabel1 += r' $(N)$'
    
        if isinstance(iwave, int):
            beta = degrees(self.wave_dir[iwave])
        else:
            # Finding the closest wave direction
            i = np.fabs(self.wave_dir - radians(iwave)).argmin()
            beta = degrees(self.wave_dir[i])
    
        title = r'%s force on body %u along mode %u for wave direction %.1f deg' % \
                (self._type_extended, ibody, iforce, beta)
    
        plt.subplot(2, 1, 1)  # TODO: faire une fonction externe pour ne pas reecrire cela pour plot_radiation ?
        plt.plot(self.omega, np.absolute(coeffs), **kwargs)
        plt.ylabel(ylabel1, fontsize=18)
        plt.title(title)
        plt.grid()
    
        plt.subplot(2, 1, 2)
        plt.plot(self.omega, np.angle(coeffs, deg=True), **kwargs)
        plt.ylabel(ylabel2, fontsize=18)
        plt.xlabel(xlabel, fontsize=18)
        plt.grid()
    
        plt.show()
    

class DiffractionDB(WaveExcitation):
    _type = 'diff'
    _type_extended = 'Diffraction'
    
    def __init__(self):
        super(DiffractionDB, self).__init__()
        
        
class FroudeKrylovDB(WaveExcitation):
    _type = 'FK'
    _type_extended = 'Froude Krylov'
    
    def __init__(self):
        super(FroudeKrylovDB, self).__init__()
        # self.depth = depth
        # self.grav = grav
        # self.x_wave_measure = x_wave_measure
        # self.y_wave_measure = y_wave_measure

    def eval(self, rho_water, grav, depth, x_wave_measure, y_wave_measure):
        #  de FroudeKrylovDB
        """Computes the Froude-Krylov complex coefficients from indident wave field"""
        # Remarque: l'evaluation est faite sur le full range de frequence (wcut est bypassed)
        # TODO: sortir le potentiel, les pressions et les vitesses normales...

        from . import wave_dispersion_relation as wdr

        omega = self.get_full_omega()
        k_wave = wdr.solve_wave_dispersion_relation(omega, depth, grav)
        # g_w = self.grav / omega
        
        body_mapper = self.body_mapper
        
        froude_krylov = np.zeros((body_mapper.nb_force_modes, self._nb_frequencies, self.nb_wave_dir),
                                 dtype=np.complex)
    
        i_global_force = 0
        for body in body_mapper.bodies:  # TODO: rendre body_mapper iterable
            # normals = body.mesh.faces_normals
            # areas = body.mesh.faces_areas
            centers = body.mesh.faces_centers
            x = centers[:, 0]
            y = centers[:, 1]
            z = centers[:, 2]
        
            # COMPUTING FIELDS
            ctheta = np.cos(self.wave_dir)  # is ntheta
            stheta = np.sin(self.wave_dir)  # is ntheta
        
            kctheta = np.einsum('i, j -> ij', k_wave, ctheta)  # is nw x ntheta
            kstheta = np.einsum('i, j -> ij', k_wave, stheta)  # is nw x ntheta
        
            kw_bar = np.einsum('i, jk -> ijk', x - x_wave_measure, kctheta)
            kw_bar += np.einsum('i, jk -> ijk', y - y_wave_measure, kstheta)  # is nf x nw x ntheta
            exp_jkw_bar = np.exp(1j * kw_bar)  # is nf x nw x ntheta
            
            if np.isinf(depth):
                # Infinite depth
                kxzph = np.einsum('i, j -> ij', z, k_wave)  # is nf x nw
                cih = np.exp(kxzph)  # is nf x nw

            else:
                # Finite depth
                kxzph = np.einsum('i, j -> ij', z + depth, k_wave)  # is nf x nw
                chkh_1 = 1. / np.cosh(k_wave * depth)  # is nw

                cih = np.einsum('ij, j -> ij', np.cosh(kxzph), chkh_1)  # is nf x nw
                sih = np.einsum('ij, j -> ij', np.sinh(kxzph), chkh_1)  # is nf x nw


        
            cih_exp_jkw_bar = np.einsum('ij, ijk -> ijk', cih, exp_jkw_bar)  # is nf x nw x ntheta
            # sih_exp_jkw_bar = np.einsum('ij, ijk -> ijk', sih, exp_jkw_bar)  # is nf x nw x ntheta
        
            # g_w_cih_exp_jkw_bar = np.einsum('j, ijk -> ijk', g_w, cih_exp_jkw_bar)  # is nf x nw x ntheta
            # g_w_sih_exp_jkw_bar = np.einsum('j, ijk -> ijk', g_w, sih_exp_jkw_bar)  # is nf x nw x ntheta
        
            # incident_potential = -1j * g_w_cih_exp_jkw_bar  # is nf x nw x ntheta
            pressure = rho_water * grav * cih_exp_jkw_bar  # is nf x nw x ntheta
        
            # i, j, k = pressure.shape
            # velocities = np.zeros((i, j, k, 3), dtype=np.complex)  # is nf x nw x ntheta x 3
            # velocities[:, :, :, 0] = np.einsum('ijk, jk -> ijk', g_w_cih_exp_jkw_bar, kctheta)
            # velocities[:, :, :, 1] = np.einsum('ijk, jk -> ijk', g_w_cih_exp_jkw_bar, kstheta)
            # velocities[:, :, :, 2] = -1j * np.einsum('j, ijk -> ijk', k_wave, g_w_sih_exp_jkw_bar)
        
            # normal_velocities = - np.einsum('ijkl, il -> ijk', velocities, normals)  # is nf x nw x ntheta
        
            for iforce in xrange(body.nb_force_modes):
                nds = body.get_nds(iforce)
                # froude_krylov[i_global_force, :, :] =  np.einsum('ijk, i -> jk', pressure, nds)
                froude_krylov[i_global_force, :, :] =  np.einsum('ijk, i -> jk', pressure, -nds) #Il s'agit de la normale entrante
                i_global_force += 1
    
        self.data = froude_krylov
    

class WaveExcitationDB(WaveExcitation):
    _type = 'exc'
    _type_extended = 'Excitation'
    
    def __init__(self):
        super(WaveExcitationDB, self).__init__()


# TODO: voir a faire une moyenne des coeffs hors diag pour le meme corps
class RadiationDB(_FreqDB):
    """Class to hold hydrodynamic radiation coefficients

    Parameters
    ----------
    wmin : float
        Minimum angular frequency (rad/s)
    wmax : float
        Maximum angular frequency (rad/s)
    nw : int
        Number of frequency samples
    ca : array_like
        array of radiation damping coefficients samples
    cm : array_like
        array of radiation added mass coefficients samples
    bodies : list
        List of HydroBody objects involved in the hydrodynamic model
    cm_inf : array_like, optional
        array of infinite frequency added mass coefficients
    """
    
    def __init__(self):
        super(RadiationDB, self).__init__()
        
        self._ca = None
        self._cm = None
        
        self._cm_inf = None

        self._irf_db = None
        self._irf_ku_db = None
        
        self.body_mapper = HydroBodySetMapping()
        self._flags = np.empty(0, dtype=np.bool)
        
    def set_data(self, ca, cm, cm_inf=None):
        
        self._ca = np.asarray(ca, dtype=np.float)
        self._cm = np.asarray(cm, dtype=np.float)
        if cm_inf is None:
            self._cm_inf = None
        else:
            self._cm_inf = np.asarray(cm_inf, dtype=np.float)
        
        nf, nw, nm = ca.shape
        
        self._flags = np.ones((nf, nm), dtype=np.bool)
    
    @property
    def radiation_damping(self):
        if self._iwcut is None:
            ca = self._ca
        else:
            ca = self._ca[:, :self._iwcut, :]  # TODO: changer l'ordre des dimensions et avoir (iforce, irad, iw)
        return np.einsum('ijk, ik -> ijk', ca, self._flags)
    
    @property
    def added_mass(self):
        if self._iwcut is None:
            cm = self._cm
        else:
            cm = self._cm[:, :self._iwcut, :]
        return np.einsum('ijk, ik -> ijk', cm, self._flags)
    
    @property
    def infinite_added_mass(self):
        if self._cm_inf is None:
            return 
        else:
            return self._cm_inf * self._flags
    
    @property
    def flags(self):
        return self._flags
    
    @flags.setter
    def flags(self, flags):
        n = self.body_mapper.nb_force_modes
        m = self.body_mapper.nb_motion_modes
        assert flags.shape == (n, m)
        self._flags = flags
    
    @property
    def frequency_response(self):
        """Returns the complex radiation frequency response (radiation transfer function)
    
        This is K(jw) = CA(w) + jw(CM(w)-CMinf).
        It is to be used in identification procedures
    
        Returns
        -------
        np.ndarray
            (nb_force_modes x nb_frequencies x nb_motion_modes) Complex array of radiation frequency responses
        """
        ca = self.radiation_damping
        cm = self.added_mass
    
        if self._cm_inf is None:
            self.eval_infinite_added_mass()
    
        cminf = self.infinite_added_mass
    
        return ca + np.einsum('j, ijk -> ijk', 1j * self.omega, cm - cminf[:, np.newaxis, :])
    
    def clean(self):  # FIXME: affiner la methode !!!!!!
        
        irf_db = self.eval_impulse_response_function()
        irf_db.clean()
        self.flags = irf_db.flags
        
    def reinit(self):
        self._flags = True
    
    def get_frequency_response(self, ibody_motion, idof, ibody_force, iforce):
        """Get the radiation frequency response for a given modes set

        Parameters
        ----------
        ibody_motion : int
            Index of the body having a motion
        idof : int
            Index of the local body's raditation mode (motion)
        ibody_force : int
            Index of the body where the radiation force is applied
        iforce : int
            Index of the local body's force mode

        Returns
        -------
        np.ndarray
            Complex array of radiation frequency response
        """
        kjw = self.frequency_response
        
        idof_gal, iforce_gal = self.body_mapper.get_general_indexes(ibody_motion, idof, ibody_force, iforce)
        
        if self._flags[idof_gal, iforce_gal]:
            return kjw[iforce_gal, :, idof_gal]
        else:
            return np.zeros(self.nb_frequencies, dtype=np.float)
    
    def get_added_mass_coeff(self, ibody_motion, idof, ibody_force, iforce):
        """Get the frequency dependent added mass coefficient

        Parameters
        ----------
        ibody_motion : int
            Index of the body having a motion
        idof : int
            Index of the local body's raditation mode (motion)
        ibody_force : int
            Index of the body where the radiation force is applied
        iforce : int
            Index of the local body's force mode

        Returns
        -------
        np.ndarray
            nb_frequencies array of Added mass frequency components
        """

        idof_gal, iforce_gal = self.body_mapper.get_general_indexes(ibody_motion, idof, ibody_force, iforce)
        if self._flags[idof_gal, iforce_gal]:
            return self.added_mass[iforce_gal, :, idof_gal]
        else:
            return np.zeros(self.nb_frequencies, dtype=np.float)

    def get_added_mass(self, ibody_force, ibody_motion):
        idof_start, iforce_start = self.body_mapper.get_general_indexes(ibody_motion, 0, ibody_force, 0)
        nb_force = self.body_mapper.get_body(ibody_force).nb_force_modes
        nb_dof = self.body_mapper.get_body(ibody_motion).nb_dof
        idof_end, iforce_end = self.body_mapper.get_general_indexes(ibody_motion, nb_dof-1, ibody_force, nb_force-1)
        return self.added_mass[iforce_start:iforce_end+1, :, idof_start:idof_end+1]

    def get_radiation_damping_coeff(self, ibody_motion, idof, ibody_force, iforce):
        """Get the frequency dependent radiation damping coefficient

        Parameters
        ----------
        ibody_motion : int
            Index of the body having a motion
        idof : int
            Index of the local body's radiation mode (motion)
        ibody_force : int
            Index of the body where the radiation force is applied
        iforce : int
            Index of the local body's force mode

        Returns
        -------
        np.ndarray
            nb_frequencies array of radiation damping frequency components
        """

        idof_gal, iforce_gal = self.body_mapper.get_general_indexes(ibody_motion, idof, ibody_force, iforce)
        if self._flags[idof_gal, iforce_gal]:
            return self.radiation_damping[iforce_gal, :, idof_gal]
        else:
            return np.zeros(self.nb_frequencies, dtype=np.float)

    def get_radiation_damping(self, ibody_force, ibody_motion):
        idof_start, iforce_start = self.body_mapper.get_general_indexes(ibody_motion, 0, ibody_force, 0)
        nb_force = self.body_mapper.get_body(ibody_force).nb_force_modes
        nb_dof = self.body_mapper.get_body(ibody_motion).nb_dof
        idof_end, iforce_end = self.body_mapper.get_general_indexes(ibody_motion, nb_dof-1, ibody_force, nb_force-1)
        return self.radiation_damping[iforce_start:iforce_end+1, :, idof_start:idof_end+1]
    
    def eval_impulse_response_function(self, tf=30., dt=None, full=True):
        """Computes the impulse response function

        It uses the Ogilvie formulas based on radiation damping integration (Inverse Fourier Transform)

        Parameters
        ----------
        tf : float, optional
            Final time (seconds). Default is 30.
        dt : float, optional
            Time step (seconds). Default is None. If None, a time step is computed according to the max
            frequency of
            hydrodynamic coefficients (the Nyquist frequency is taken)
        full : bool, optional
            If True (default), it will use the full frequency range for computations.
            
        Returns
        -------
        RadiationIRFDB:
            Radiation Impulse Response Database object
        """
        
        irf_db = RadiationIRFDB()
        
        if dt is None:
            # Using Shannon theorem
            dt = pi / (10 * self.max_frequency)
    
        # TODO: Determiner le temps de coupure global
    
        time = np.arange(start=0., stop=tf + dt, step=dt)
    
        tf = time[-1]  # It is overwriten !!
        
        if full:
            w = self.get_full_omega()
        else:
            w = self.omega
            
        wt = np.einsum('i, j -> ij', w, time)  # is nw x nt
        cwt = np.cos(wt)  # is nw x nt
        
        if full:
            ca = np.einsum('ijk, ik -> ijk', self._ca, self._flags)
        else:
            ca = self.radiation_damping
        
        kernel = np.einsum('ijk, jl -> ijkl', ca, cwt)  # is nb_forces x nb_motions x nt
    
        irf_data = (2 / pi) * np.trapz(kernel, x=w, axis=1)  # is nb_forces x nb_motions x nt
    
        irf_db.set_data(tf, dt, irf_data)
        irf_db.body_mapper = self.body_mapper

        self._irf_db = irf_db

        return irf_db

    def get_irf_db(self):
        if self._irf_db is None:
            self.eval_impulse_response_function(tf=100, dt=0.1)
        return self._irf_db

    def eval_impulse_response_function_Ku(self, tf=30., dt=None, full=True):
        """Computes the impulse response function relative to the ship advance speed

        ref : F. Rongère et al. (Journées de l'Hydrodynamique 2010 - Nantes)

        Parameters
        ----------
        tf : float, optional
            Final time (seconds). Default is 30.
        dt : float, optional
            Time step (seconds). Default is None. If None, a time step is computed according to the max
            frequency of
            hydrodynamic coefficients (the Nyquist frequency is taken)
        full : bool, optional
            If True (default), it will use the full frequency range for computations.

        Returns
        -------
        RadiationIRFDB:
            Radiation Impulse Response Database object
        """

        self.eval_infinite_added_mass()     # FIXME : pourquoi cela ne fonctionne par en extérieur

        irf_db = RadiationIRFDB()

        if dt is None:
            # Using Shannon theorem
            dt = pi / (10. * self.max_frequency)

        time = np.arange(start=0., stop=tf + dt, step=dt)

        tf = time[-1]

        if full:
            w = self.get_full_omega()
        else:
            w = self.omega

        wt = np.einsum('i, j ->ij', w, time)  # is nw x nt
        cwt = np.cos(wt)  # is nw x nt

        if full:
            cm = np.einsum('ijk, ik -> ijk', self._cm, self._flags)
        else:
            cm = self.added_mass

        cm_inf = self.infinite_added_mass

        cm_diff = np.zeros(cm.shape)
        for j in range(w.size):
            cm_diff[:, j, :] = cm_inf[:, :] - cm[:, j, :]

        cm_diff[:, :, 4] = -cm_diff[:, :, 2]
        cm_diff[:, :, 5] = cm_diff[:, :, 1]

        kernel = np.einsum('ijk, jl -> ijkl', cm_diff, cwt)  # is nb_forces x nb_motions x nt

        irf_data = (2. / pi) * np.trapz(kernel, x=w, axis=1)

        irf_db.set_data(tf, dt, irf_data)
        irf_db.body_mapper = self.body_mapper

        self._irf_ku_db = irf_db

        return irf_db

    def get_irf_ku(self):
        if self._irf_ku_db is None:
            self.eval_impulse_response_function_Ku(tf=100, dt=0.1)
        return self._irf_ku_db

    def eval_infinite_added_mass(self, full=True):
        """Evaluates the infinite added mass matrix coefficients using Ogilvie formula.
        
        Parameters
        ----------
        full : bool, optional
            If True (default), it will use the full frequency range for computations.
        
         It uses the Ogilvie formula to get the coefficients from the impulse response functions
         
        """
        irf_db = self.get_irf_db()
        
        if full:
            w = self.get_full_omega()
            cm = self._cm
        else:
            w = self.omega
            cm = self.added_mass
            
        time = irf_db.time
    
        wt = np.einsum('i, j -> ij', w, time)  # is nw x nt
        sin_wt = np.sin(wt)
    
        irf = irf_db.data  # is nint x nrad x nt
    
        kernel = np.einsum('ijk, lk -> iljk', irf, sin_wt)  # is nint x nw x nrad x nt
        integral = np.einsum('ijk, j -> ijk', np.trapz(kernel, x=time, axis=3), 1. / w)  # is nint x nw x nrad
    
        self._cm_inf = (cm + integral).mean(axis=1)  # is nint x nrad
        return self._cm_inf

    def get_infinite_added_mass_matrix(self, ibody_force=None, ibody_motion=None):
        """Get the infinite added mass matrix for forces on one body and motions of a possibly other body

        Parameters
        ----------
        ibody_force : int, optional
            Index of body on which radiation forces applies. If None (default), every lines of the system will be taken
        ibody_motion : int, optional
            Index of body that performs motions. If None (default), every degree of freedom of the system will be taken

        Returns
        -------
        np.ndarray

        Notes
        -----
        * It both index are used and are equal, the 'pure' added mass matrix of a body is returned
        * If both index are used but are different, coupling matrices are returned
        * If only the first index is given, then lines of the returned matrix correspond to forces on the
          corresponding body but columns concern every degree of freedom of the system
        * If only the second index is given, then lines of the returned matrix concern forces on every body of the
          structure but columns only concern the degree of freedom of the specified body
        """
        if ibody_force is None:
            i1, i2 = 0, self.body_mapper.nb_force_modes
        else:
            body_force = self.body_mapper.bodies[ibody_force]
            i1, i2 = body_force.force_modes[0].general_index, body_force.force_modes[-1].general_index + 1
    
        if ibody_motion is None:
            j1, j2 = 0, self.body_mapper.nb_motion_modes
        else:
            body_motion = self.body_mapper.bodies[ibody_motion]
            j1, j2 = body_motion.motion_modes[0].general_index, body_motion.motion_modes[-1].general_index + 1

        result = self.infinite_added_mass

        return result[i1:i2, j1:j2]
    
    def plot(self, ibody_motion, idof, ibody_force, iforce, **kwargs):
        """Plots the radiation coefficients of a given modes set

        Parameters
        ----------
        ibody_motion : int
            Index of the body having a motion
        idof : int
            Index of the local body's raditation mode (motion)
        ibody_force : int
            Index of the body where the radiation force is applied
        iforce : int
            Index of the local body's force mode
        kwargs: optional
            Arguments that are to be used by pyplot
        """
        if self._cm_inf is None:
            has_cminf = False
        else:
            has_cminf = True
        
        ca = self.get_radiation_damping_coeff(ibody_motion, idof, ibody_force, iforce)
        cm = self.get_added_mass_coeff(ibody_motion, idof, ibody_force, iforce)
        cminf = None
        
        if has_cminf:
            idof_gal, iforce_gal = self.body_mapper.get_general_indexes(ibody_motion, idof, ibody_force, iforce)
            cminf = self._cm_inf[iforce_gal, idof_gal]
    
        xlabel = r'$\omega (rad/s)$'
        ylabel1 = r'$CA(\omega)$'
        ylabel2 = r'$CM(\omega)$'
    
        # force_mod = self.bodies[ibody_force].force_modes[iforce]
        # motion_mod = self.bodies[ibody_motion].motion_modes[idof]
        
        force_mod = self.body_mapper.get_force_mode(ibody_force, iforce)
        motion_mod = self.body_mapper.get_motion_mode(ibody_motion, idof)
    
        if isinstance(force_mod, ForceMode):
            force_str = 'force'
            if isinstance(motion_mod, TranslationMode):
                ylabel1 += r' $(kg/s)$'
                ylabel2 += r' $(kg)$'
                motion_str = 'translation'
            else:  # Rotation
                ylabel1 += r' $(kg\,m/s)$'
                ylabel2 += r' $(kg\,m)$'
                motion_str = 'rotation'
        else:
            force_str = 'moment'
            if isinstance(motion_mod, TranslationMode):
                ylabel1 += r' $(kg\,m/s)$'
                ylabel2 += r' $(kg\,m)$'
                motion_str = 'translation'
            else:
                ylabel1 += r' $(kg\,m^2/s)$'
                ylabel2 += r' $(kg\,m^2)$'
                motion_str = 'rotation'
    
        title = r"Radiation coefficients giving %s on body %u along mode %u " \
                r"for a %s motion of body %u along mode %u" \
                % (force_str, ibody_force, iforce, motion_str, ibody_motion, idof)
    
        w = self.omega
    
        plt.subplot(2, 1, 1)
        plt.plot(w, ca, **kwargs)
        plt.ylabel(ylabel1, fontsize=18)
        plt.title(title)
        plt.grid()
    
        plt.subplot(2, 1, 2)
        plt.plot(w, cm, **kwargs)
        
        if has_cminf:
            w_01 = [w[0], w[-1]]
            cm_inf01 = [cminf, cminf]
            plt.plot(w_01, cm_inf01, **kwargs)
        plt.ylabel(ylabel2, fontsize=18)
        plt.xlabel(xlabel, fontsize=18)
        plt.grid()
    
        plt.show()

    def _plot_radiation_array(self, coeff):
        """Generic method to plot an array of radiation coefficients

        Parameters
        ----------
        coeff : str
            Must be 'added_mass' or 'damping'

        Returns
        -------

        """
        assert coeff in ('added_mass', 'damping')
    
        omega = self.omega
        
        has_cminf = False
        cminf = None
        w_01 = None
        
        if coeff == 'added_mass':
            data = self.added_mass
            suptitle = "Added mass array"
            
            if self._cm_inf is not None:
                has_cminf = True
                cminf = self.infinite_added_mass
                w_01 = [omega[0], omega[-1]]
            
            is_added_mass = True
        else:
            data = self.radiation_damping
            suptitle = "Radiation damping array"
            is_added_mass = False
    
        n_integration = self.body_mapper.nb_force_modes
        n_radiation = self.body_mapper.nb_motion_modes
    
        iplot = 1
        for iforce_mod in xrange(n_integration):
            for idof_mod in xrange(n_radiation):
                plt.subplot(n_integration, n_radiation, iplot)
                plt.plot(omega, data[iforce_mod, :, idof_mod])
                
                if iforce_mod == 0:  # Putting labels on the left
                    ibody, imod = self.body_mapper._motion_mod_dict[idof_mod]
                    plt.title('(%u, %u)' % (ibody, imod))
                
                if idof_mod == 0:  # Putting labels on the top
                    ibody, imod = self.body_mapper._force_mod_dict[iforce_mod]
                    plt.ylabel('(%u, %u)' % (ibody, imod))
            
                if is_added_mass and has_cminf:
                    c = cminf[iforce_mod, idof_mod]
                    plt.plot(w_01, [c, c])
            
                iplot += 1
    
        plt.suptitle(suptitle)
        plt.show()

    def plot_added_mass_array(self):
        """Plots the added mass coefficients in an array for every modes"""
        self._plot_radiation_array('added_mass')

    def plot_radiation_damping_array(self):
        """Plots the radiation damping coefficients in an array for every modes"""
        self._plot_radiation_array('damping')
    

class RadiationIRFDB(object):
    # TODO: meme en cas d'unique force et mouvement, data devrait de shape (1, 1, nt) !!
    # NOTE: data is of shape nb_forces x nb_motion x nb_time_samples
    
    # (nb_force_modes x nb_motion_modes x nb_time_irf)
    
    def __init__(self):
        self._tf = 0.
        self._dt = 0.
        self.data = np.empty(0, dtype=np.float)
        
        self.body_mapper = HydroBodySetMapping()
        
        self._flags = np.empty(0, dtype=np.bool)

    def set_data(self, tf, dt, data):
        self._tf = tf
        self._dt = dt
        
        data = np.asarray(data, dtype=np.float)
        assert data.shape[2] == len(self.time), "Inconsistent number of sample in IRF"
        self.data = data
        
        self._flags = np.ones(data.shape[:2], dtype=np.bool)

    @property
    def irf(self):
        return self.data

    @property
    def tmax(self):
        return self._tf

    @property
    def dt(self):
        return self._dt
    
    @property
    def is_empty(self):
        if self.data is None:
            return True
        else:
            return False
    
    @property
    def nb_force_modes(self):
        return self.data.shape[0]
    
    @property
    def nb_motion_modes(self):
        return self.data.shape[1]
    
    @property
    def nb_time_samples(self):
        return self.data.shape[2]
    
    @property
    def time(self):
        dt = self._dt
        return np.arange(0., self._tf + dt, dt, dtype=np.float)

    @property
    def flags(self):
        return self._flags

    @flags.setter
    def flags(self, flags):
        n = self.body_mapper.nb_force_modes
        m = self.body_mapper.nb_motion_modes
        assert flags.shape == (n, m)
        self._flags = flags
    
    def clean(self):
        max_irf = self.data.max(axis=-1)
    
        # data = np.fabs(irf_db.data)
        # data = irf_db.data
        # integ = np.fabs(np.trapz(data, x=irf_db.time, axis=-1)**2)
    
        # plt.matshow(integ, cmap=plt.cm.inferno)
        # plt.show()
    
        # maxval = integ.max()
        # print integ
        self._flags[max_irf < 1e4] = False  # TODO: les seuils doivent etre reglables
    
    def convolve(self, velocities):
        # TODO: verifier qu'on envoie bien le bon nombre de vitesses
        assert velocities.ndim == 2
            
        integrand = np.einsum('ijk, jk -> ijk', self.data[..., ::-1], velocities)
        return np.squeeze(np.trapz(integrand, dx=self._dt, axis=2))
    
    def get_impulse_response_function(self, ibody_motion, idof, ibody_force, iforce):
        """Get the impulse response function for a given modes set

        Parameters
        ----------
        ibody_motion : int
            Index of the body having a motion
        idof : int
            Index of the local body's raditation mode (motion)
        ibody_force : int
            Index of the body where the radiation force is applied
        iforce : int
            Index of the local body's force mode

        Returns
        -------
        np.ndarray
        """

        idof_gal = self.body_mapper.get_general_motion_index(ibody_motion, idof)
        iforce_gal = self.body_mapper.get_general_force_index(ibody_force, iforce)

        return self.data[iforce_gal, idof_gal, :]

    def get_impulse_response(self, ibody_force, ibody_motion):
        idof_start, iforce_start = self.body_mapper.get_general_indexes(ibody_motion, 0, ibody_force, 0)
        nb_force = self.body_mapper.get_body(ibody_force).nb_force_modes
        nb_dof = self.body_mapper.get_body(ibody_motion).nb_dof
        idof_end, iforce_end = self.body_mapper.get_general_indexes(ibody_motion, nb_dof-1, ibody_force, nb_force-1)
        return self.data[iforce_start:iforce_end+1, idof_start:idof_end+1, :]



    def plot(self, ibody_motion, idof, ibody_force, iforce, **kwargs):
        """Plots the impulse response function of a given modes set

        Parameters
        ----------
        ibody_motion : int
            Index of the body having a motion
        idof : int
            Index of the local body's raditation mode (motion)
        ibody_force : int
            Index of the body where the radiation force is applied
        iforce : int
            Index of the local body's force mode
        kwargs: optional
            Arguments that are to be used by pyplot
        """
    
        data = self.get_impulse_response_function(ibody_motion, idof, ibody_force, iforce)
        plt.plot(self.time, data, **kwargs)
        plt.xlabel(r'$t (s)$', fontsize=18)
        plt.ylabel(r'$K(t)$', fontsize=18)  # TODO: mettre une unite
        plt.title('Impulse response function of force %u on body %u for a motion %u of body %u' %
                  (iforce, ibody_force, idof, ibody_motion))
        plt.grid()
        plt.show()
        
    def plot_array(self):
        """Plots the impulse response functions in an array for every modes"""

        time = self.time

        n_force = self.body_mapper.nb_force_modes
        ndof = self.body_mapper.nb_motion_modes

        iplot = 1
        for iforce_mod in xrange(n_force):
            for idof_mod in xrange(ndof):
                plt.subplot(n_force, ndof, iplot)
                plt.plot(time, self.data[iforce_mod, idof_mod, :])
                iplot += 1
                
        plt.suptitle('Impulse response functions')
        plt.show()

    
class KochinFunction(_FreqDB):
    def __init__(self):
        super(KochinFunction, self).__init__()
        raise NotImplementedError


# TODO : classe a retirer
        
class NemohBEMData(object):  # FIXME: n'est plus utilise !!! A RETIRER DES QUE TOUTES LES METHODES SERONT MIGREES
    """Class to manage a Hydrodynamic database for several bodies

    At that moment, it only deals with 1st order hydrodynamics but in the future, it should also deal with second
    order (mean drift, QTF)

    Attributes
    ----------
    dirname : str
        The root directory where the BEM computations have taken place
    rho_water : float
        Water density (kg/m**3)
    grav : float
        Gravity acceleration (m/s**2)
    depth : float
        Water depth (meters)
    x_wave_measure : float
        x position of free surface elevation measure
    y_wave_measure : float
        y position of free surface elevation measure
    nb_frequencies : int
        Number of frequency discretization in BEM computations
    min_frequency : float
        Min frequency (rad/s) of frequency discretization. Should usually not be 0.
    max_frequency : float
        Max frequency of frequency (rad/s) discretization.
    nb_wave_directions : int
        Number of wave directions in BEM computations
    min_wave_dir : float
        Min wave direction (deg) of waves
    max_wave_dir : float
        Max wave direction (def) of waves
    diffraction : np.array
        (nb_force_modes x nb_frequencies x nb_motion_modes) complex numbers array of wave diffraction coefficients
    added_mass : np.ndarray
        (nb_force_modes x nb_frequencies x nb_motion_modes) array of added mass coefficients
    radiation_damping : np.ndarray
        (nb_force_modes x nb_frequencies x nb_motion_modes) array of radiation damping coefficients
    radiation_flags : np.ndarray
        (nb_force_modes x nb_motion_mods) boolean array of hydrodynamic modes that are significant (True if significant)
    _irf : RadiationIRFDB
         Impulse response functions of the system
    dt_irf : float
        Time step to take to compute impulse response functions (seconds)
    tf_irf : float
        Final time to take to compute impulse response functions (seconds)
    has_kochin : bool
        True if we load Kochin functions
    kochin_radiation : np.ndarray
        (nb_motion_modes x nb_frequencies x nb_dir_kochin) complex array of Kochin functions for radiation modes
    kochin_diffraction : np.ndarray
        (nb_wave_dir x nb_frequencies x nb_dir_kochin) complex array of Kochin functions for diffraction modes
    nb_dir_kochin : int
        Number of directions used to evaluate Kochin functions
    min_dir_kochin : float
        Min angle (deg) used to evaluate Kochin functions
    max_dir_kochin : float
        Max angle (deg) used to evaluate Kochin functions
    free_surface : np.ndarray
        Not used
    nx_fs : int
        Not used
    ny_fs : int
        Not used
    xlim_fs : float
        Not used
    ylim_fs : float
        Not used
    bodies : list
        List of HydroBody objects taken in BEM computations
        
    Note:
    -----
    Instanciate the method and just load Nemoh computations using the set_calulation_file('Nemoh.cal') method.
    """
    def __init__(self):
        self.dirname = ''
        
        # General parameters
        self.rho_water = 1000.
        self.grav = 9.81
        self.depth = inf
        self.x_wave_measure = 0.
        self.y_wave_measure = 0.
        
        # Waves  # TODO: ou mettre les infos de vague (principalement diffraction)
        self.nb_wave_directions = 1
        self.min_wave_dir = 0.
        self.max_wave_dir = 0.
        
        # Hydrodynamic database
        self.diffraction = None
        self._froude_krylov = None
        
        self.radiation_db = None
        
        # Frequency  # TODO: ou mettre les infos de frequence (radiation + diffraction)?
        self.nb_frequencies = 1
        self.min_frequency = 0.
        self.max_frequency = 0.
        
        # TODO: faire disparaitre ces donnees une fois RadiationDB implemente
        self.added_mass = None
        self._infinite_added_mass = None
        self.radiation_damping = None
        self.radiation_flags = None  # TODO: utiliser ce tableau (etabli dans clean_radiation) afin de
        # dynamiquement annuler les coeffs non significatifs
        
        self._irf = RadiationIRFDB()  # TODO: utiliser un objet
        self.has_kochin = False
        self.kochin_radiation = None  # TODO: utiliser un objet
        self.kochin_diffraction = None
        self.nb_dir_kochin = 0
        self.min_dir_kochin = 0.
        self.max_dir_kochin = 0.
        self.free_surface = None
        self.nx_fs = 0
        self.ny_fs = 0
        self.xlim_fs = 0.
        self.ylim_fs = 0.

        self.bodies = HydroBodySetMapping()
        
        self._force_mod_dict = dict()
        self._motion_mod_dict = dict()
    
    @property
    def kochin_directions(self):  # TODO: methode de KochinFcn
        """Angles array taken for Kochin functions computations
        
        Returns
        -------
        np.ndarray
            angles array in radians
        """
        return np.radians(np.linspace(self.min_dir_kochin, self.max_dir_kochin, self.nb_dir_kochin))
    
    @property
    def fs_grid(self):  # TODO: methode de HydroDB
        """Grid taken to compute the free surface elevation from BEM computations
        
        Returns
        -------
        np.ndarray, np.ndarray
            A tuple abtained by meshgrid
        """
        x = np.linspace(0., self.xlim_fs, self.nx_fs)
        y = np.linspace(0., self.ylim_fs, self.ny_fs)
        return np.meshgrid(x, y, indexing='ij')  # TODO: voir l'indexing correspondant a Nemoh
    
# TODO: fin de classe a retirer
    
    
class HydroBody(object):
    """Class to represent a hydrodynamic body
    
    It is used to represent a hydrodynamic body that is used in BEM computations. This class is mainly here to
    enclose integration and radiation modes on a per body fashion.
    """
    def __init__(self, ibody, mesh):
        self.ibody = ibody
        self.mesh = mesh
        self.motion_modes = list()
        self.force_modes = list()
        
        self._nds = None
    
    def append_motion_mode(self, motion):
        assert isinstance(motion, (TranslationMode, RotationMode))
        self.motion_modes.append(motion)
        
    def append_force_mode(self, force):
        assert isinstance(force, (ForceMode, MomentMode))
        self.force_modes.append(force)
    
    def get_motion_mode(self, idof):
        """Get the body's motion mode based on its index
        
        Parameters
        ----------
        idof : int
            Index of the mode inside the body's mode definition

        Returns
        -------
        TranslationMode, RotationMode
        """
        assert idof >= 0, "Motion mode for body %u must lie between 0 and %u" \
                          % (self.ibody, self.nb_dof - 1)
        try:
            return self.motion_modes[idof]
        except IndexError:
            raise IndexError('Body %u has %u motion modes. Attempted to access to mode %u which is out of range'
                             % (self.ibody, self.nb_dof, idof))
    
    def get_force_mode(self, i):
        """Get the body's integration mode based on its index

        Parameters
        ----------
        i : int
            Index of the mode inside the body's mode definition

        Returns
        -------
        IntegrationModeForce, IntegrationModeMoment
        """
        assert i >= 0, "Force mode for body %u must lie between 0 and %u" \
                       % (self.ibody, self.nb_force_modes - 1)
        try:
            return self.force_modes[i]
        except IndexError:
            raise IndexError('Body %u has %u force modes. Attempted to access to mode %u which is out of range'
                             % (self.ibody, self.nb_force_modes, i))
    
    @property
    def nb_dof(self):
        """The total body's number of motion modes"""
        return len(self.motion_modes)
    
    @property
    def nb_force_modes(self):
        """The total body's number of force modes"""
        return len(self.force_modes)
    
    def _compute_nds(self):
        """Computes the term n dS for each force mode of the body"""
        
        self._nds = np.zeros((self.nb_force_modes, self.mesh.nb_faces), dtype=np.float)
        
        areas = self.mesh.faces_areas
        normals = self.mesh.faces_normals
        centers = self.mesh.faces_centers
        
        for i, force_mode in enumerate(self.force_modes):
            if isinstance(force_mode, ForceMode):
                self._nds[i, :] = areas * np.einsum('ij, j -> i', normals, force_mode.direction)
            else:  # Moment
                am = centers - force_mode.point
                vel = np.cross(force_mode.direction, am)
                self._nds[i, :] = areas * (normals * vel).sum(axis=1)
    
    def get_nds(self, iforce):
        """Get the array of ndS . direction vector for the specified force mode
        
        Parameters
        ----------
        iforce : int
            Force mode index

        Returns
        -------
        np.ndarray
            (n_faces) Array of ndS quantities
        """
        if self._nds is None:
            self._compute_nds()
        
        return self._nds[iforce, :]
        
        
class _Mode(object):
    """Base class to define modes (motion or force)
    
    Parameters
    ----------
    direction : array_like
        3 component array of the mode direction
    point : array_like
        3 component array of the mode point
    general_index : int
        Index of the mode in the general structure
    ibody : int
        Index of the body that belongs that mode
    """
    def __init__(self, direction, point, general_index, ibody):
        self.direction = np.asarray(direction, dtype=np.float)
        self.point = np.asarray(point, dtype=np.float)
        self.general_index = general_index
        self.ibody = ibody
        
        
class TranslationMode(_Mode):
    """Class representing a translation mode (radiation)"""
    def __init__(self, direction, point, general_index, ibody):
        super(TranslationMode, self).__init__(direction, point, general_index, ibody)
    
    def __str__(self):
        d = self.direction
        return "Motion mode of body %u in translation (direction: %.3f, %.3f, %.3f)" % (self.ibody, d[0], d[1], d[2])
        
        
class RotationMode(_Mode):
    """Class representing a rotation mode (radiation)"""
    def __init__(self, direction, point, general_index, ibody):
        super(RotationMode, self).__init__(direction, point, general_index, ibody)

    def __str__(self):
        d = self.direction
        p = self.point
        return "Motion mode of body %u in rotation (direction: %.3f, %.3f, %.3f, point: %.3f, %.3f, %.3f)" % \
               (self.ibody, d[0], d[1], d[2], p[0], p[1], p[2])


class ForceMode(_Mode):
    """Class representing a pure force mode"""
    def __init__(self, direction, point, general_index, ibody):
        super(ForceMode, self).__init__(direction, point, general_index, ibody)

    def __str__(self):
        d = self.direction
        return "Force mode of body %u in force (direction: %.3f, %.3f, %.3f)" % (self.ibody, d[0], d[1], d[2])


class MomentMode(_Mode):
    """Class representing a moment mode"""
    def __init__(self, direction, point, general_index, ibody):
        super(MomentMode, self).__init__(direction, point, general_index, ibody)

    def __str__(self):
        d = self.direction
        p = self.point
        return "Force mode of body %u in moment (direction: %.3f, %.3f, %.3f, point: %.3f, %.3f, %.3f)" % \
               (self.ibody, d[0], d[1], d[2], p[0], p[1], p[2])


class UnknownMotionMode(Exception):
    def __init__(self, i, cal_file):
        self.type = i
        self.cal_file = cal_file
        
    def __str__(self):
        return "Unknown Motion Mode %u from Nemoh calculation file %s" % (self.type, self.cal_file)


class UnknownForceMode(Exception):
    def __init__(self, i, cal_file):
        self.type = i
        self.cal_file = cal_file
        
    def __str__(self):
        return "Unknown Force Mode %u from Nemoh calculation file %s" % (self.type, self.cal_file)
        

class HydroBodySetMapping(object):
    def __init__(self):
        self.bodies = list()
        self._motion_mod_dict = dict()
        self._force_mod_dict = dict()
    
    def append(self, body):
        assert isinstance(body, HydroBody)
        self.bodies.append(body)
    
    @property
    def nb_bodies(self):
        return len(self.bodies)
    
    @property
    def nb_motion_modes(self):
        """Total number of radiation modes of the BEM computations (motion modes)

        Returns
        -------
        int
        """
        n = 0
        for body in self.bodies:
            n += body.nb_dof
        return n

    @property
    def nb_force_modes(self):
        """Total number of integration modes of the BEM computations (force modes)

        Returns
        -------
        int
        """
        n = 0
        for body in self.bodies:
            n += body.nb_force_modes
        return n

    @property
    def body(self):
        return self.bodies

    def get_body(self, ibody):  # TODO: Methode de HydroBodySetMapping
        """Get the HydroBody object specified by its number

        Parameters
        ----------
        ibody : int
            Index of the HydroBody

        Returns
        -------
        HydroBody
        """
        assert ibody >= 0, "Body index must be positive and currently lie between 0 and %u" % self.nb_bodies
        try:
            return self.bodies[ibody]
        except IndexError:
            raise IndexError("Current data contain only %u bodies. Attempted to access body number %u"
                             % (self.nb_bodies, ibody))

    def get_general_motion_index(self, ibody, idof):  # TODO: Methode de HydroBodySetMapping
        try:
            return self.bodies[ibody].motion_modes[idof].general_index
        except IndexError:
            raise IndexError('Body number or local motion id is out of range.')

    def get_general_force_index(self, ibody, iforce):  # TODO: Methode de HydroBodySetMapping
        try:
            return self.bodies[ibody].force_modes[iforce].general_index
        except IndexError:
            raise IndexError('Body number or local force id is out of range.')
    
    def get_general_indexes(self, ibody_motion, idof, ibody_force, iforce):
        idof_gal = self.get_general_motion_index(ibody_motion, idof)
        iforce_gal = self.get_general_force_index(ibody_force, iforce)
        return idof_gal, iforce_gal
    
    def get_force_mode(self, ibody, iforce):
        return self.bodies[ibody].force_modes[iforce]
    
    def get_motion_mode(self, ibody, idof):
        return self.bodies[ibody].motion_modes[idof]
