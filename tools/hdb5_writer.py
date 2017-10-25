#!/usr/bin/env python
#  -*- coding: utf-8 -*-
import frydom.hydrodynamics.bem.hydro_db
import h5py
import numpy as np
import datetime

from bemio.data_structures.bem import HydrodynamicData

def write_hdb5(hydro_db, out_file=None):
    
    if out_file is None:
        out_file = 'frydom_hdb.h5'
        
    with h5py.File(out_file, 'w') as f:
        
        # Date
        dset = f.create_dataset('CreationDate', data=str(datetime.datetime.now()))
        dset.attrs['Description'] = "Date of creation of this database"
        
        # Gravity acceleration
        dset = f.create_dataset('GravityAcc', data=hydro_db.grav)
        dset.attrs['Unit'] = 'm/s**2'
        dset.attrs['Description'] = "Gravity acceleration"
        
        # Water density
        dset = f.create_dataset('WaterDensity', data=hydro_db.rho_water)
        dset.attrs['Unit'] = 'kg/m**3'
        dset.attrs['Description'] = 'Water Density'
        
        # Normalisation length
        dset = f.create_dataset('NormalizationLength', data=1.)
        dset.attrs['Unit'] = 'm'
        dset.attrs['Description'] = 'Normalization length'
        
        # Water height
        water_depth = hydro_db.depth
        if water_depth == 'infinite':
            water_depth = 0.
        dset = f.create_dataset('WaterDepth', data=water_depth)
        dset.attrs['Unit'] = 'm'
        dset.attrs['Description'] = 'Water depth. It is 0. for infinite values and positive for finite values'
        
        # Number of interacting bodies
        nbBodies = hydro_db.body_mapper.nb_bodies
        dset = f.create_dataset('NbBody', data=nbBodies)
        dset.attrs['Description'] = 'Number of interacting bodies'
        
        # Frequency discretization
        dis_group = "/discretizations"
        f.create_group(dis_group)
        freq_group = dis_group + "/frequency"
        nw = hydro_db.nb_frequencies
        dset = f.create_dataset(freq_group + "/nbFrequencies", data=nw)
        dset.attrs['Description'] = "Number of frequencies in the discretization"
        dset = f.create_dataset(freq_group + "/minFrequency", data=hydro_db.min_frequency)
        dset.attrs['Unit'] = "rad/s"
        dset.attrs['Description'] = "Minimum frequency specified for the computations"
        dset = f.create_dataset(freq_group + "/maxFrequency", data=hydro_db.nb_frequencies)
        dset.attrs['Unit'] = "rad/s"
        dset.attrs['Description'] = "Maximum frequency specified for the computations"
        
        # Wave direction discretization
        wave_dir_group = dis_group + "/wave_directions"
        nb_wave_dir = hydro_db.nb_wave_dir
        dset = f.create_dataset(wave_dir_group + "/nbWaveDirections", data=nb_wave_dir)
        dset.attrs['Description'] = "Number of wave directions in the discretization"
        min_wave_dir = hydro_db.min_wave_dir
        dset = f.create_dataset(wave_dir_group + "/minAngle", data=min_wave_dir)
        dset.attrs['Unit'] = "deg"
        dset.attrs['Description'] = "Minimum angle specified for the computations"
        max_wave_dir = hydro_db.max_wave_dir
        dset = f.create_dataset(wave_dir_group + "/maxAngle", data=max_wave_dir)
        dset.attrs['Unit'] = "deg"
        dset.attrs['Description'] = "Maximum angle specified for the computations"
        
        irf_db = hydro_db.radiation_db.eval_impulse_response_function(100, dt=0.1)
        
        # Time discretization
        time_dir_group = dis_group + "/time"
        time = irf_db.time
        nt = len(time)
        dset = f.create_dataset(time_dir_group + "/nbTimeSample", data=nt)
        dset.attrs['Description'] = "Number of time samples"
        
        dset = f.create_dataset(time_dir_group + "/finalTime", data=time[-1])
        dset.attrs['Unit'] = "s"
        dset.attrs['Description'] = "Final time for the impulse response function"
        
        
        for body in hydro_db.body_mapper.bodies:
            # assert isinstance(body, HydrodynamicData)
            
            group_name = 'Body_%u' % body.ibody
            
            dset = f.create_group(group_name)

            # Body name
            dset = f.create_dataset(group_name + "/BodyName", data=body.mesh.name)
            dset.attrs['Description'] = "Body name"
            
            # Id of the body
            dset = f.create_dataset(group_name + "/ID", data=body.ibody)
            dset.attrs['Description'] = "Body identifier"
            
            # Position of the body
            dset = f.create_dataset(group_name + "/BodyPosition",
                                    data=np.zeros(3, dtype=np.float))
            dset.attrs['Unit'] = 'm'
            dset.attrs['Description'] = "Position of the body in the absolute frame"
            
            # TODO: gerer ici la definition des modes de force et de mouvement
            
            dset = f.create_dataset(group_name + "/nbForceModes", data=body.nb_force_modes)
            dset.attrs['Description'] = "Number of force modes for body number %u" % body.ibody
            
            for imode, force_mode in enumerate(body.force_modes):
                mode_group = group_name + "/force_modes/mode_%u" % imode
                f.create_group(mode_group)
                f.create_dataset(mode_group + "/direction", data=force_mode.direction)
                
                if isinstance(force_mode, frydom.hydrodynamics.bem.hydro_db.ForceMode):
                    f.create_dataset(mode_group + "/type", data='force')
                
                elif isinstance(force_mode, frydom.hydrodynamics.bem.hydro_db.MomentMode):
                    f.create_dataset(mode_group + "/type", data='moment')
                    f.create_dataset(mode_group + "/application_point", data=force_mode.point)

            dset = f.create_dataset(group_name + "/nbMotionModes", data=body.nb_dof)
            dset.attrs['Description'] = "Number of motion modes for body number %u" % body.ibody
            
            for idof, motion_mode in enumerate(body.motion_modes):
                mode_group = group_name + "/motion_modes/mode_%u" % idof
                f.create_group(mode_group)
                f.create_dataset(mode_group + "/direction", data=motion_mode.direction)
                
                if isinstance(motion_mode, frydom.hydrodynamics.bem.hydro_db.TranslationMode):
                    f.create_dataset(mode_group + "/type", data='translation')
                
                elif isinstance(motion_mode, frydom.hydrodynamics.bem.hydro_db.RotationMode):
                    f.create_dataset(mode_group + "/type", data='rotation')
                    f.create_dataset(mode_group + "/rotation_point", data=motion_mode.point)
            
            # Mesh file
            dset = f.create_dataset(group_name + "/mesh/nbVertices", data=body.mesh.nb_vertices)
            dset = f.create_dataset(group_name + "/mesh/vertices", data=body.mesh.vertices)
            dset = f.create_dataset(group_name + "/mesh/nbFaces", data=body.mesh.nb_faces)
            dset = f.create_dataset(group_name + "/mesh/faces", data=body.mesh.faces)
            
            # Froude-Krylov
            fk_db = hydro_db.froude_krylov_db
            fk_group = group_name + "/excitation/froude_krylov"
            f.create_group(fk_group)
            wave_dirs = np.linspace(min_wave_dir, max_wave_dir, nb_wave_dir)
            for idir, wave_dir in enumerate(wave_dirs):
                fk_group_angle = fk_group + "/angle_%u" % idir
                f.create_group(fk_group_angle)
                dset = f.create_dataset(fk_group_angle + "/angle", data=wave_dir)
                dset.attrs['Unit'] = 'deg'
                dset.attrs['Description'] = "Wave direction angle of the data"

                dset = f.create_dataset(fk_group_angle + "/realCoeffs", data=fk_db.data[:, :, idir].real)
                dset.attrs['Unit'] = ''
                dset.attrs['Description'] = "Real part of the Froude-Krylov hydrodynamic coefficients for %u forces " \
                                            "on body %u as a function of frequency" % (body.nb_force_modes, body.ibody)
                dset = f.create_dataset(fk_group_angle + "/imagCoeffs", data=fk_db.data[:, :, idir].imag)
                dset.attrs['Unit'] = ''
                dset.attrs['Description'] = "Imaginary part of the Froude-Krylov hydrodynamic coefficients for %u " \
                                            "forces on body %u as a function of frequency" % (body.nb_force_modes, body.ibody)
            
            # Diffraction excitation
            diff_db = hydro_db.diffraction_db
            diff_group = group_name + "/excitation/diffraction"
            f.create_group(diff_group)
            for idir, wave_dir in enumerate(wave_dirs):
                diff_group_angle = diff_group + "/angle_%u" % idir
                f.create_group(diff_group_angle)
                f.create_dataset(diff_group_angle + "/angle", data=wave_dir)
                dset.attrs['Unit'] = 'deg'
                dset.attrs['Description'] = "Wave direction angle of the data"
                
                f.create_dataset(diff_group_angle + "/realCoeffs", data=diff_db.data[:, :, idir].real)
                dset.attrs['Unit'] = ''
                dset.attrs['Description'] = "Real part of the diffraction hydrodynamic coefficients for %u forces " \
                                            "on body %u as a function of frequency" % (body.nb_force_modes, body.ibody)
                f.create_dataset(diff_group_angle + "/imagCoeffs", data=diff_db.data[:, :, idir].imag)
                dset.attrs['Unit'] = ''
                dset.attrs['Description'] = "Imaginary part of the diffraction hydrodynamic coefficients for %u forces " \
                                            "on body %u as a function of frequency" % (body.nb_force_modes, body.ibody)
            
            # Radiation
            
            rad_db = hydro_db.radiation_db
            rad_group = group_name + "/radiation"
            f.create_group(rad_group)
            
            added_mass = rad_db.added_mass
            wave_damping = rad_db.radiation_damping
            
            for body_j in hydro_db.body_mapper.bodies:
                jbody = body_j.ibody
                rad_body_j_group = rad_group + "/Body_%u_motion" % jbody
                
                dg = f.create_group(rad_body_j_group)
                dg.attrs['Description'] = "Hydrodynamic coefficients for motion of body %u that radiates waves and " \
                                          "generate forces on body %u" % (jbody, body.ibody)

                rad_body_j_CM_group = rad_body_j_group + "/added_mass"
                dg = f.create_group(rad_body_j_CM_group)
                dg.attrs['Description'] = "Added mass coefficients for acceleration of body %u that radiates waves " \
                                          "and generates forces on body %u" % (jbody, body.ibody)

                rad_body_j_CA_group = rad_body_j_group + "/wave_damping"
                dg = f.create_group(rad_body_j_CA_group)
                dg.attrs['Description'] = "Wave damping coefficients for velocity of body %u that radiates waves " \
                                          "and generates forces on body %u" % (jbody, body.ibody)
                
                
                for imode, motion_mode_j in enumerate(body_j.motion_modes):
                    # Infinite added mass



                    # Added mass
                    dset = f.create_dataset(rad_body_j_CM_group + "/Body_%u_DOF_%u" % (jbody, imode),
                                            data=added_mass[:, :, imode])
                    dset.attrs['Unit'] = ""
                    dset.attrs['Description'] = "Added mass coefficients for an acceleration of body %u and force on " \
                                                "body %u (nbForce x nw)" % (jbody, body.ibody)
                    
                    # Wave damping
                    dset = f.create_dataset(rad_body_j_CA_group + "/Body_%u_DOF_%u" % (jbody, imode),
                                            data=wave_damping[:, :, imode])
                    dset.attrs['Unit'] = ""
                    dset.attrs['Description'] = "Wave damping coefficients for an acceleration of body %u and force " \
                                                "on body %u (nbForce x nw)" % (jbody, body.ibody)
                
                

# TODO: faire un outil en ligne de commande !!
if __name__ == '__main__':

    import argparse

    

    
    from frydom.hydrodynamics.bem.bem_reader import NemohReader
    
    import cPickle
    
    
    
    
    from bemio.io import nemoh
    sim_dir = '/home/frongere/Documents/Cylinder'

    reader = NemohReader(cal_file=sim_dir + "/Nemoh.cal")
    

    write_hdb5(reader.hydro_db)
