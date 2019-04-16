.. _hydrostatics:

Hydrostatic force
-----------------

Three levels of approximation can be defined for the hydrostatic restoring force:

- a linear model;
- a weakly nonlinear model;
- a fully nonlinear model.

.. _linear_hydrostatics:

Linear model
************

The generalized hydrostatic static force expression for the linear approximation is given by:

.. math::
    \mathbf{f}_H(t) = -\mathbf{K}_h \mathbf{x}(t)

where

- :math:`\mathbf{x}` is the generalized position vector, with respect to the equilibrium frame;
- :math:`\mathbf{K}_h` is the hydrostatic stiffness matrix.

The stiffness coefficients for the horizontal degrees of freedom (surge, sway and yaw) are null. The coefficients
for the heave, roll and pitch degrees of freedom are specified with respect to the :any:`equilibrium frame <equilibrium_frame>`.

.. note::
    The hydrostatic stiffness matrix is an input of the numerical model, considered constant during the simulation.

.. _weakly_nonlinear_hydrostatics:

Weakly nonlinear model
**********************

On contrary to the linear model, the exact position of the body is now considered. However, the incident wave is still
supposed to be of small curvature; the free surface can then be represented by the plane :math:`z = 0`.

As a consequence to these two approximations, the hydrostatic load is now computed by integrating the hydrostatic pressure
over the wetted body surface :math:`S_0`, defined by the exact position of the body mesh, clipped by the plane :math:`z = 0`.

.. math::
    \mathbf{f}_H(t)= -\iint_{S_0} \rho gz \mathbf{n} dS

where

- :math:`\rho` is the water density;
- :math:`g` denotes the gravity constant;
- :math:`\mathbf{n}` is normal vector, pointing outward the body surface.

.. note::
    The hydrostatic force is computed at the center of buoyancy and then transported to the center of gravity, to yield
    the hydrostatic torque.

.. _nonlinear_hydrostatics:

Fully nonlinear model
*********************

The assumption of small incident wave curvature is no longer considered; the free surface wave elevation is then :math:`z = \eta_I`.
The wetted body surface is now the exact wetted body surface, :math:`S_I`, defined by the exact position of the body mesh,
and delimited by the incident wave field.

.. math::
    \mathbf{f}_H(t) = -\iint_{S_I} \rho gz \mathbf{n} dS

.. note::
    The computation of the hydrostatic torque follows the same principle as in the weakly nonlinear model.