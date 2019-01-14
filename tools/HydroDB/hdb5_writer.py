#!/usr/bin/env python
#  -*- coding: utf-8 -*-
# import frydom.hydrodynamics.bem.hydro_db
from . import hydro_db
import h5py
import numpy as np
import datetime


def symetrize(wave_dirs, fk_db, diff_db):

    [nmode, nbody, ndir] = fk_db.data.shape

    for i in range(ndir):

        if wave_dirs[i] > np.float32(0.):

            # New wave direction
            new_dir = -wave_dirs[i] % 360
            if new_dir < 0:
                new_dir += 360.

            # Add corresponding data
            wave_dirs = np.append(wave_dirs, new_dir)

            fk_db_temp = np.copy(fk_db.data[:, :, i])
            fk_db_temp[(1, 4, 5), :] = -fk_db_temp[(1, 4, 5), :]

            fk_db.data = np.concatenate((fk_db.data, fk_db_temp.reshape(nmode, nbody, 1)), axis=2)

            diff_db_temp = np.copy(diff_db.data[:, :, i])
            diff_db_temp[(1, 4, 5), :] = -diff_db_temp[(1, 4, 5), :]
            diff_db.data = np.concatenate((diff_db.data, diff_db_temp.reshape(nmode, nbody, 1)), axis=2)

    return wave_dirs, fk_db, diff_db


def write_hdb5(hdb, out_file=None):
    
    if out_file is None:
        out_file = 'frydom_hdb.h5'
        
    with h5py.File(out_file, 'w') as f:

        # Date
        dset = f.create_dataset('CreationDate', data=str(datetime.datetime.now()))
        dset.attrs['Description'] = "Date of creation of this database"
        
        # Gravity acceleration
        dset = f.create_dataset('GravityAcc', data=hdb.grav)
        dset.attrs['Unit'] = 'm/s**2'
        dset.attrs['Description'] = "Gravity acceleration"
        
        # Water density
        dset = f.create_dataset('WaterDensity', data=hdb.rho_water)
        dset.attrs['Unit'] = 'kg/m**3'
        dset.attrs['Description'] = 'Water Density'
        
        # Normalisation length
        dset = f.create_dataset('NormalizationLength', data=1.)
        dset.attrs['Unit'] = 'm'
        dset.attrs['Description'] = 'Normalization length'
        
        # Water height
        water_depth = hdb.depth
        if water_depth == 'infinite':
            water_depth = 0.
        dset = f.create_dataset('WaterDepth', data=water_depth)
        dset.attrs['Unit'] = 'm'
        dset.attrs['Description'] = 'Water depth. It is 0. for infinite values and positive for finite values'
        
        # Number of interacting bodies
        nbBodies = hdb.body_mapper.nb_bodies
        dset = f.create_dataset('NbBody', data=nbBodies)
        dset.attrs['Description'] = 'Number of interacting bodies'
        
        # Frequency discretization
        discretization_path = "/Discretizations"
        f.create_group(discretization_path)
        frequential_path = discretization_path + "/Frequency"

        nw = hdb.nb_frequencies
        dset = f.create_dataset(frequential_path + "/NbFrequencies", data=nw)
        dset.attrs['Description'] = "Number of frequencies in the discretization"

        dset = f.create_dataset(frequential_path + "/MinFrequency", data=hdb.min_frequency)
        dset.attrs['Unit'] = "rad/s"
        dset.attrs['Description'] = "Minimum frequency specified for the computations"

        dset = f.create_dataset(frequential_path + "/MaxFrequency", data=hdb.max_frequency)
        dset.attrs['Unit'] = "rad/s"
        dset.attrs['Description'] = "Maximum frequency specified for the computations"

        fk_db = hdb.froude_krylov_db
        diff_db = hdb.diffraction_db

        # Wave direction discretization
        wave_direction_path = discretization_path + "/WaveDirections"

        wave_dirs = np.linspace(hdb.min_wave_dir, hdb.max_wave_dir, hdb.nb_wave_dir)

        # --- Adjust convention of wage direction to GOTO
        if hdb.min_wave_dir >= -np.float32() and hdb.max_wave_dir <= 180. + np.float32():
            wave_dirs = 180. - wave_dirs
            wave_dirs, fk_db, diff_db = symetrize(wave_dirs, fk_db, diff_db)

        else:
            wave_dirs = np.fmod(wave_dirs + 180., 360.)

        n180 = 0
        i360 = -9
        for idir in range(wave_dirs.size):
            wave_dir = wave_dirs[idir]

            if abs(wave_dir) < 0.01:
                i360 = idir
            elif abs(wave_dir - 180) < 0.01:
                n180 += 1
                if n180 == 2:
                    wave_dirs[idir] = 360.
                    fk_db.data[:, :, idir] = fk_db.data[:, :, i360]
                    diff_db.data[:, :, idir] = diff_db.data[:, :, i360]

        # -- sort direction
        sort_dirs = np.argsort(wave_dirs)
        wave_dirs = wave_dirs[sort_dirs]
        fk_db.data = fk_db.data[:, :, sort_dirs]
        diff_db.data = diff_db.data[:, :, sort_dirs]

        # write data
        nb_wave_dir = wave_dirs.shape[0]
        dset = f.create_dataset(wave_direction_path + "/NbWaveDirections", data=nb_wave_dir)
        dset.attrs['Description'] = "Number of wave directions in the discretization"

        min_wave_dir = np.min(wave_dirs)
        dset = f.create_dataset(wave_direction_path + "/MinAngle", data=min_wave_dir)
        dset.attrs['Unit'] = "deg"
        dset.attrs['Description'] = "Minimum angle specified for the computations"

        max_wave_dir = np.max(wave_dirs)
        dset = f.create_dataset(wave_direction_path + "/MaxAngle", data=max_wave_dir)
        dset.attrs['Unit'] = "deg"
        dset.attrs['Description'] = "Maximum angle specified for the computations"

        # TODO: regler dans le writer
        irf_db = hdb.radiation_db.eval_impulse_response_function(tf=100, dt=0.1)
        irf_ku_db = hdb.radiation_db.eval_impulse_response_function_Ku(tf=100, dt=0.1)

        # Time discretization
        irf_time_path = discretization_path + "/Time"
        time = irf_db.time
        nt = len(time)
        dset = f.create_dataset(irf_time_path + "/NbTimeSample", data=nt)
        dset.attrs['Description'] = "Number of time samples"
        
        dset = f.create_dataset(irf_time_path + "/FinalTime", data=time[-1])
        dset.attrs['Unit'] = "s"
        dset.attrs['Description'] = "Final time for the impulse response function"
        
        
        for body in hdb.body_mapper.bodies:
            # assert isinstance(body, HydrodynamicData)
            
            body_path = '/Bodies/Body_%u' % body.ibody
            
            dset = f.create_group(body_path)

            # Body name
            dset = f.create_dataset(body_path + "/BodyName", data=body.mesh.name)
            dset.attrs['Description'] = "Body name"
            
            # Id of the body
            dset = f.create_dataset(body_path + "/ID", data=body.ibody)
            dset.attrs['Description'] = "Body identifier"
            
            # Position of the body
            dset = f.create_dataset(body_path + "/BodyPosition",
                                    data=np.zeros(3, dtype=np.float))
            dset.attrs['Unit'] = 'm'
            dset.attrs['Description'] = "Position of the body in the absolute frame"
            
            # TODO: gerer ici la definition des modes de force et de mouvement

            body_modes_path = body_path + "/Modes"
            dset = f.create_dataset(body_modes_path + "/NbForceModes", data=body.nb_force_modes)
            dset.attrs['Description'] = "Number of force modes for body number %u" % body.ibody
            
            for imode, force_mode in enumerate(body.force_modes):
                mode_path = body_modes_path + "/ForceModes/Mode_%u" % imode
                f.create_group(mode_path)
                f.create_dataset(mode_path + "/Direction", data=force_mode.direction)
                
                if isinstance(force_mode, hydro_db.ForceMode):
                    f.create_dataset(mode_path + "/Type", data='LINEAR')
                
                elif isinstance(force_mode, hydro_db.MomentMode):
                    f.create_dataset(mode_path + "/Type", data='ANGULAR')
                    f.create_dataset(mode_path + "/Point", data=force_mode.point)

            dset = f.create_dataset(body_modes_path + "/NbMotionModes", data=body.nb_dof)
            dset.attrs['Description'] = "Number of motion modes for body number %u" % body.ibody
            
            for idof, motion_mode in enumerate(body.motion_modes):
                mode_path = body_modes_path + "/MotionModes/Mode_%u" % idof
                f.create_group(mode_path)
                f.create_dataset(mode_path + "/Direction", data=motion_mode.direction)
                
                if isinstance(motion_mode, hydro_db.TranslationMode):
                    f.create_dataset(mode_path + "/Type", data='LINEAR')
                
                elif isinstance(motion_mode, hydro_db.RotationMode):
                    f.create_dataset(mode_path + "/Type", data='ANGULAR')
                    f.create_dataset(mode_path + "/Point", data=motion_mode.point)
            
            # Mesh file
            mesh_path = body_path + "/Mesh"
            dset = f.create_dataset(mesh_path + "/NbVertices", data=body.mesh.nb_vertices)
            dset = f.create_dataset(mesh_path + "/Vertices", data=body.mesh.vertices)
            dset = f.create_dataset(mesh_path + "/NbFaces", data=body.mesh.nb_faces)
            dset = f.create_dataset(mesh_path + "/Faces", data=body.mesh.faces)

            # Froude-Krylov
            excitation_path = body_path + "/Excitation"


            fk_group = excitation_path + "/FroudeKrylov"
            f.create_group(fk_group)
      

            for idir, wave_dir in enumerate(wave_dirs):
                wave_dir_path = fk_group + "/Angle_%u" % idir
                f.create_group(wave_dir_path)
                wave_dir_path = fk_group + "/Angle_%u" % idir
                dset = f.create_dataset(wave_dir_path + "/Angle", data=wave_dir)
                dset.attrs['Unit'] = 'deg'
                dset.attrs['Description'] = "Wave direction angle of the data"

                dset = f.create_dataset(wave_dir_path + "/RealCoeffs", data=fk_db.data[:, :, idir].real)
                dset.attrs['Unit'] = ''
                dset.attrs['Description'] = "Real part of the Froude-Krylov hydrodynamic coefficients for %u forces " \
                                            "on body %u as a function of frequency" % (body.nb_force_modes, body.ibody)
                dset = f.create_dataset(wave_dir_path + "/ImagCoeffs", data=fk_db.data[:, :, idir].imag)
                dset.attrs['Unit'] = ''
                dset.attrs['Description'] = "Imaginary part of the Froude-Krylov hydrodynamic coefficients for %u " \
                                            "forces on body %u as a function of frequency" % (body.nb_force_modes, body.ibody)
            
            # Diffraction excitation

            diffraction_path = excitation_path + "/Diffraction"
            f.create_group(diffraction_path)
            for idir, wave_dir in enumerate(wave_dirs):
                wave_dir_path = diffraction_path + "/Angle_%u" % idir
                f.create_group(wave_dir_path)
                f.create_dataset(wave_dir_path + "/Angle", data=wave_dir)
                dset.attrs['Unit'] = 'deg'
                dset.attrs['Description'] = "Wave direction angle of the data"
                
                f.create_dataset(wave_dir_path + "/RealCoeffs", data=diff_db.data[:, :, idir].real)
                dset.attrs['Unit'] = ''
                dset.attrs['Description'] = "Real part of the diffraction hydrodynamic coefficients for %u forces " \
                                            "on body %u as a function of frequency" % (body.nb_force_modes, body.ibody)
                f.create_dataset(wave_dir_path + "/ImagCoeffs", data=diff_db.data[:, :, idir].imag)
                dset.attrs['Unit'] = ''
                dset.attrs['Description'] = "Imaginary part of the diffraction hydrodynamic coefficients for %u forces " \
                                            "on body %u as a function of frequency" % (body.nb_force_modes, body.ibody)
            
            # Radiation
            
            rad_db = hdb.radiation_db
            radiation_path = body_path + "/Radiation"
            f.create_group(radiation_path)

            rad_db.eval_infinite_added_mass()  # FIXME: ici on redeclenche un calcul de reponses impulsionnelles...

            # added_mass = rad_db.added_mass  # FIXME: ici, on veut recuperer la matrice pour le couple de corps i, j !!
            # wave_damping = rad_db.radiation_damping # FIXME: pareil ici !!



            # impulse_response_function = rad_db.irf # FIXME: pareil ici !!
            # irf_db.plot_array()

            
            for body_j in hdb.body_mapper.bodies:
                jbody = body_j.ibody


                radiation_body_motion_path = radiation_path + "/BodyMotion_%u" % jbody
                
                dg = f.create_group(radiation_body_motion_path)
                dg.attrs['Description'] = "Hydrodynamic coefficients for motion of body %u that radiates waves and " \
                                          "generate forces on body %u" % (jbody, body.ibody)

                added_mass_path = radiation_body_motion_path + "/AddedMass"
                dg = f.create_group(added_mass_path)
                dg.attrs['Description'] = "Added mass coefficients for acceleration of body %u that radiates waves " \
                                          "and generates forces on body %u" % (jbody, body.ibody)

                radiation_damping_path = radiation_body_motion_path + "/RadiationDamping"
                dg = f.create_group(radiation_damping_path)
                dg.attrs['Description'] = "Wave damping coefficients for velocity of body %u that radiates waves " \
                                          "and generates forces on body %u" % (jbody, body.ibody)

                irf_path = radiation_body_motion_path + "/ImpulseResponseFunction"
                dg = f.create_group(irf_path)
                dg.attrs['Description'] = "Impulse response function for velocity of body %u that radiates waves " \
                                          "and generates forces on body %u" % (jbody, body.ibody)

                irf_ku_path = radiation_body_motion_path + "/ImpulseResponseFunctionKu"
                dg = f.create_group(irf_ku_path)
                dg.attrs['Description'] = "Impulse response function Ku for velocity of body %u that radiates waves " \
                                          "and generates forces on body %u" % (jbody, body.ibody)

                # Infinite added mass for body i and motion of body j
                dset = f.create_dataset(radiation_body_motion_path + "/InfiniteAddedMass",
                                        data=rad_db.get_infinite_added_mass_matrix(body.ibody, jbody))
                dset.attrs['Description'] = "Infinite added mass matrix that modifies the apparent mass of body %u from " \
                                            "acceleration of body %u" % (body.ibody, jbody)

                added_mass = rad_db.get_added_mass(body.ibody, jbody)
                radiation_damping = rad_db.get_radiation_damping(body.ibody, jbody)
                impulse_response_function = irf_db.get_impulse_response(body.ibody, jbody)
                impulse_response_function_ku = irf_ku_db.get_impulse_response_Ku(body.ibody, jbody)

                for imode, motion_mode_j in enumerate(body_j.motion_modes):

                    # Added mass
                    dset = f.create_dataset(added_mass_path + "/DOF_%u" % (imode),
                                            data=added_mass[:, :, imode])
                    dset.attrs['Unit'] = ""
                    dset.attrs['Description'] = "Added mass coefficients for an acceleration of body %u and force on " \
                                                "body %u (nbForce x nw)" % (jbody, body.ibody)
                    
                    # Wave damping
                    dset = f.create_dataset(radiation_damping_path + "/DOF_%u" % (imode),
                                            data=radiation_damping[:, :, imode])
                    dset.attrs['Unit'] = ""
                    dset.attrs['Description'] = "Wave damping coefficients for an acceleration of body %u and force " \
                                                "on body %u (nbForce x nw)" % (jbody, body.ibody)

                    # Impulse response function
                    dset = f.create_dataset(irf_path + "/DOF_%u" % (imode),
                                            data=impulse_response_function[:, imode, :])
                    dset.attrs['Description'] = "Impulse response functions"

                    # Impulse response function Ku
                    dset = f.create_dataset(irf_ku_path + "/DOF_%u" % (imode),
                                            data=impulse_response_function_ku[:, imode, :])
                    dset.attrs['Description'] = "Impulse response functions Ku"