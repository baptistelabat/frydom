.. _Hydrostatics_bench:


Hydrostatics and stability
~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~


Hydrostatic equilibrium
-----------------------

The hydrostatic equilibrium of a floating body can be solved within FRyDoM by mainly two methods.
The first method is based on the direct time domain decay simulation of the body motions with proper damping models, or
relaxation (zeroing the linear and angular velocities). The second methods relies on a Newton-Raphson solver, which
solves iteratively the hydrostatic equilibrium, by computing the stiffness matrix on the body clipped mesh.


.. toctree::
    :maxdepth: 2

    hydrostatics_decay
    hydrostatics_equilibrium


Stability analysis
------------------

.. toctree::
    :maxdepth: 2

    stability_analysis
