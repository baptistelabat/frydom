.. _hydrostatics:

Hydrostatic force
-----------------

Three models of computation of the hydrostatic loads are present:
- : a linear model;
- : a weakly nonlinear model;
- : a fully nonlinear model.

.. _linear_hydrostatics:

Linear model
************

The generalized hydrostatic static force expression for the linear approximation is given by:

.. math::
    \mathbf{f}_H(t) = -\mathbf{K}_h \mathbf{x}(t)

where

- :math:`\mathbf{x}` is the generalized position vector, with respect to the equilibrium frame;
- :math:`\mathbf{K}_h` is the hydrostatic stiffness matrix.

The stiffness coefficients for the horizontal degrees of freedom (surge, sway and yaw) are zero. The coefficients
for the heave, roll and pitch degrees of freedom are specified with respect to the :any:`equilibrium frame <equilibrium_frame>`.

This hydrostatic stiffness matrix is given in input of the numerical model and is constant during the simulation.

Weakly nonlinear model
**********************

In the linear model, the mesh used for the computation of the hydrostatic stiffness matrix is fixed. In the weakly nonlinear model, the loads are computed by the hydrostatic pressure integration over the body mesh at its real position. The free surface is represented by the plane :math:`z = 0`. Consequently, at every evaluation of the hydrostatic loads, the body mesh is clipped by the plane :math:`z = 0`. The expression of the hydrostatic force is:

.. math::
    \mathbf{f}_H(t)= -\iint_{S_0} \rho gz \mathbf{n} dS

where

- :math:`\rho` is the water density;
- :math:`g` denotes the gravity constant;
- :math:`\mathbf{n}` is normal vector, pointing outward the body surface;
- :math:`S_0` representes the wetted body surface delimited by the plane :math:`z = 0`.

This force, applied at the center of buoyancy, is transport to the center of gravity to evaluate the hydrostatic torque.

Fully nonlinear model
*********************

In the fully nonlinear model, the mesh used for the pressure integration is delimited by the incident wave field :math:`z = \eta_I` and not the plane :math:`z = 0` as in the weakly nonlinear model. Consequently, the expression of the hydrostatic force becomes:

.. math::
    \mathbf{f}_H(t) = -\iint_{S_I} \rho gz \mathbf{n} dS

where :math:`S_I` is the wetted body surface delimited by the incident wave field :math:`z = \eta_I`.

The computation of the hydrostatic torque follows the same principle as in the weakly nonlinear model.