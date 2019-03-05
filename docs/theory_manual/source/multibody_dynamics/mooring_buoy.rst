.. mooring_buoy:

Mooring buoy
============

Mooring buoys are easy, pre-defined bodies, with already built-in model forces : non linear hydrostatic and linear damping forces.
They are based on a spheric shape for their inertia distribution, collision box and visualization asset.
Their CoG is defined at the center of the sphere. Their radius and density are to be specified at creation.

Hydrostatic force
-----------------

They come with a simplified non linear hydrostatic force :

.. math::
        \mathbf{f}_{HS} (t) = - \rho_{water} V(t) \mathbf{g}

where

- :math:`\rho_{water}` is the water density,
- :math:`\mathbf{g}` is the gravity acceleration,
- :math:`V(t) = \dfrac{\pi}{3} \left[ h(t) \times \left(3 \times R^2 - h(t)^2 \right) + 2 R^3 \right]` is the immersed volume of the buoy,
- :math:`h(t)` is the immersed draft of the buoy.

This force is applied on the sphere CoG, which means it induce no torque.


Damping force
-------------

Mooring buoys posses a linear damping force, which diagonal damping coefficients must be specified at creation.
See :any:`Other damping<_other_damping>` for more information.