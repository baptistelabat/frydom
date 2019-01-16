.. _forces:

Force models
=================

Hydrodynamic forces
~~~~~~~~~~~~~~~~~~~

Hydrostatic force
-----------------

Excitation force
----------------

Radiation force
---------------

Wave drift force
----------------

Model forces
~~~~~~~~~~~~
ITTC resistance force
---------------------
.. toctree::
    :maxdepth: 1

    ITTC78

Morison force
-------------

Originally designed for vertical cylinder, the morison equation represents the inline force on a body in oscillatory flow.
It corresponds to the sum of an inertia term, due to the acceleration of the fluid particule, and a drag term, due to the flow velocity.
When the body is moving, this equation can be extended to integrate the force due to the body acceleration.

For a flow velocity :math:`u(t)`, the morison equation is given by:

.. math::
	F_M = \rho (1 + C_a) v_a (\dot{u}(t) - \ddot{X}(t)) + \frac{1}{2} C_d \rho L (u - \dot{X})|u - \dot{X}|

where

- :math:`\rho` is the water density,
- :math:`C_a` is the added mass,
- :math:`C_d`is the drag coefficient,
- :math:`L` the length of the morison element (in m),
- :math:`v_a` the volume of the morison element (in m³),
- :math:`X` the position of the morison element in global coordinate system.


Flow force (wind and current)
-----------------------------
.. toctree::
    :maxdepth: 1

    flow_force

Damping forces
--------------
.. toctree::
    :maxdepth: 1

    damping

