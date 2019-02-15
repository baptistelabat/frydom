Tutorial
========

1) Python script Template
-------------------------

Starting from an empty Python script, it is necessary to include two libraries:

.. code-block:: python

    from HDF5 import HDB5
    import numpy as np

A main function has to be defined:

.. code-block:: python
   
   def main():

The script ends with:

.. code-block:: python

    if __name__ = "__main__":
       main()

The name of the script along with its location are unimportant. Every directory for both the input and output files will be specified in the script.

**All the commands presented in the next sections will have to be implemented within the main function.**

2) HDB5 structure
-----------------

The main class of *Nemoh2HDB* is named *HDB5*. It stores and post-processes all the data resulted from *Nemoh*. This object needs to be initialized:

.. code-block:: python
   
   database = HDB5()

3) Reading *Nemoh* input and output files
-----------------------------------------

For reading the *Nemoh* input and output files along with post-processing its data, the following subroutine has to be called:

.. code-block:: python
   
   database.nemoh_reader(path_to_Nemoh_cal,nb_faces_by_wavelength)

where:
 - *path_to_Nemoh_cal* represents the path to the *.cal* input file of *Nemoh*, by default path_to_Nemoh_cal = '.';
 - *nb_faces_by_wavelength* denotes the number of faces per wave length, by default equal to 10.


4) Discretization
-----------------

To set the parameters for the new discretization of the impulse response functions, the two following parameters have to be specified:

.. code-block:: python
   
    database.discretization.nb_wave_directions = 15
    database.discretization.nb_frequencies = 50

In this case, the new discretization will have 15 wave directions and 50 frequencies.

5) Hydrostatics
---------------

Prior to setting the hydrostatic stiffness matrix of the bodies, it is necessary to creates a body structure. For instance, in case of a single body:

.. code-block:: python
   
   body = database.body[0]

Then, the hydrostatics may be activated:

.. code-block:: python
    
    body.activate_hydrostatic()

And the hydrostatic matrix defined using:

 - coefficients:

.. code-block:: python
	
    body.hydrostatic.k33 = k33
    body.hydrostatic.k44 = k44
    body.hydrostatic.k55 = k55
    body.hydrostatic.k34 = k34
    body.hydrostatic.k35 = k35
    body.hydrostatic.k45 = k45
..

 - diagonal and off-diagonal terms:

.. code-block:: python
	
    body.hydrostatic.diagonal = [k33, k44, k55]
    body.hydrostatic.non_diagonal = [k34,k35,k45]
..

 - a matrix:

.. code-block:: python
	
    body.hydrostatic.matrix = np.array([[k33,k34,k35],[k43,k44,k45],[k53,k54,k55]])

The values of k33, k34, ..., k55 must be given.

6) Wave drift
-------------

To take into account the wave drift, polar coefficients needs to be given. They depend on both the frequency and the wave direction.

If a body structure was not initialized, follow the first step of the previous section. To active the wave drift, the command is:

.. code-block:: python
    
    body.activate_wave_drift()

The frequency discretization and the polar coefficients may be written as follow (here for 4 different frequencies):

.. code-block:: python

    freqs = np.array([f1, f2, f3, f4])
    data = np.array([coeff_1, coeff_2, coeff_3, coeff_4])

Then the angular discretization has to be give (here for an angle *theta*):

.. code-block:: python

    body.wave_drift.add_cx(freqs, data, theta, unit_dir='deg', unit_freq='rads')

This previous line needs to be written as many as the number of angular discretization (by changing the value of *theta*).

For doing a new discretization with respect to the frequency, it is necessary to give a new number of frequencies (here 10):

.. code-block:: python

    body.wave_drift.nb_frequencies = 10

7) Plots
--------

Several quantities may be plotted:

 - Added mass and damping coefficients:

.. code-block:: python

    database._hdb.radiation_db.plot(ibody_motion, idof, ibody_force, iforce)

Where:
    * *ibody_motion* is the index of the moving body;
    * *idof* represents the local body radiation mode;
    * *ibody_force* denotes the body where the radiation force is applied;
    * *iforce* is the local body force mode.

All the added mass and damping coefficients may be displayed in one time:

.. code-block:: python
    
    database._hdb.radiation_db.plot_added_mass_array()
    database._hdb.radiation_db.plot_radiation_damping_array()
..

 - Exciting loads:

.. code-block:: python

    database._hdb.wave_excitation_db.plot(ibody, iforce, iwave)

with:
    * *ibody* the index of the body;
    * *iforce* the index of the body force mode;
    * *iwave* the index of the wave direction.

This subroutine may be also used with the diffraction loads and the Froude-Krylov loads:

.. code-block:: python

    database._hdb.diffraction_db.plot(ibody, iforce, iwave)
    database._hdb.froude_krylov_db.plot(ibody, iforce, iwave)
..

 - Impulse response functions:

.. code-block:: python

    database._hdb.radiation_db.get_irf_db().plot(ibody_motion, idof, ibody_force, iforce)

The definition of the input parameters is the same as for the added mass and damping coefficients.

 - Impulse response functions relative to the ship advance speed:

.. code-block:: python

    database._hdb.radiation_db.get_irf_ku().plot(ibody_motion, idof, ibody_force, iforce)

The definition of the input parameters is the same as for the added mass and damping coefficients.

8) Filtering of the impulse response functions
----------------------------------------------

To improve the quality of the impulse response functions, it is possible to filter them using a cutoff scaling function, the following command may be used:

.. code-block:: python
    
    database._initialize()
    body.cutoff_scaling_irf_k(tc, i_body_motion, i_force, i_dof, auto_apply)

where:
    * *tc* is the cutting time;
    * *i_body_motion* represents the index of the moving body;
    * *i_force* denotes the index of the force of the moving body;
    * *i_dof* is index of the dof of the moving body;
    * *auto_apply* indicates if the application of the filtering is automatic or not (by default auto_apply = False).

9) Writing the *.hdb5*
----------------------

The *.hdb5* is generated using the following subroutine:

.. code-block:: python
	
    database.write_hdb5(path_and_name_to_hdb5_file)

where *path_and_name_to_hdb5_file* represents the path including the name with its extension of the *.hdb5* file. 

By default, path_and_name_to_hdb5_file = 'frydom.hdb5'.

10) Running *Nemoh2HDB*
-----------------------

To run *Nemoh2HDB*, use the following command:

.. code-block:: python
   
   python Name_of_my_Python_file.py








