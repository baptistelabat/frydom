.. _hydrodynamic_forces:

Linear potential flow theory
~~~~~~~~~~~~~~~~~~~~~~~~~~~~

The linear potential flow theory is a common approach for seakeeping analysis of offshore structures. It is based on the
following assumptions: incompressible and irrotational flow and non viscous fluid.

By assuming small amplitude motions of the body, the position of the body is linearized around an equilibrium position, defined in FRyDoM by the  :any:`equilibrium frame <equilibrium_frame>`. 
For offshore structure with no forward speed, this position is the position of the body at the equilibrium state.
For offshore structure with non-zero forward speed, the equilibrium frame follows the constant steady velocity of the structure.
:any:`The following figure<fig_equilibrium_frame>` represents the instantaneous position of the body and the equilibrium frame position.

.. _fig_equilibrium_frame:
.. figure:: _static/equilibrium_frame.png
    :align: center
    :alt: Equilibrium frame

    Representation of the instantaneous position of the ship (grey) and equilibrium position (white)

By assuming small wave amplitude, the free surface is linearized around the mean sea level `z = 0`.

In time domain, the dynamic of the body due to wave loads is represented by the Cummin’s equation:

.. math::
    \left( \mathbf{M} + \mathbf{A}_{\infty} (U) \right) \mathbf{\ddot{x}}(t) + \mathbf{B}_{\infty}(U) \mathbf{\dot{x}}(t) + \int_0^t \mathbf{K}(t-\tau,U) \mathbf{\dot{x}}(\tau) d\tau + \mathbf{K}_h \mathbf{x} = \mathbf{f}_E(t) + \mathbf{f}_{EXT}(t)

where

- :math:`\mathbf{x}` is the generalized position vector in respect to the equilibrium frame;
- :math:`U` is the steady forward speed of the body;
- :math:`\mathbf{M}` is the generalized mass matrix of the body;
- :math:`\mathbf{A}_{\infty}` and :math:`\mathbf{B}_{\infty}` are the infinite added mass and damping coefficients;
- :math:`\mathbf{K}` is the impulse response function;
- :math:`\mathbf{K}_h` is the stiffness matrix;
- :math:`\mathbf{f}_e` is the generalized wave excitation force;
- :math:`\mathbf{f}_{ext}` is the generalized external force.

Hydrostatic force
-----------------

Three models of computation of the hydrostatic loads are present:
- : a linear model;
- : a weakly nonlinear model;
- : a fully nonlinear model.

Linear model
************

The generalized hydrostatic static force expression for the linear approximation is given by:

.. math::
    \mathbf{f}_H(t) = -\mathbf{K}_h \mathbf{x}(t)

where

- :math:`\mathbf{x}` is the generalized position vector, with respect to the equilibrium frame;
- :math:`\mathbf{K}_h` is the hydrostatic stiffness matrix.

The stiffness coefficients for the horizontal degrees of freedom (surge, sway and yaw) are zero. The coefficients
for the heave, roll and pitch degrees of freedom are specified with respect to the equilibrium frame.

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

Froude-Krylov force
-------------------

The Froude-Krylov loads are due to the integration of the pressure of the incident wave field over the wetted body surface. Several models are available, following the same decomposition presented for the hydrostatic loads:
- : a linear model;
- : a weakly nonlinear model;
- : a fully nonlinear model.

Linear model
************

The Froude-Krylov force, given by the linear approximation, is:

.. math::
    \mathbf{f}_{FK}(t) = \sum_m \sum_n \Im\left(A_{mn} \mathbf{f}_{fk}(\omega_m,\theta_n) e^{j(k_m\bar{x}_n - \omega_m^e t + \Phi_{mn})}\right)

with

- :math:`\omega_m` the wave frequency;
- :math:`\theta_n` the wave direction;
- :math:`\bar{x}_n = x \cos(\theta_n) + y \sin(\theta_n)`;
- :math:`\omega_m^e = \omega_m - k_mU` is the encounter circular frequency, calculated using the steady forward speed;
- :math:`A_{mn}` the wave amplitude for the wave frequency :math:`\omega_m` and the wave direction :math:`\theta_n`;
- :math:`\mathbf{f}_{fk}(\omega_m,\theta_n)` is the frequency-domain Froude-Krylov force component (with both the diffraction and Froude-Krylov components) for the wave frequency :math:`\omega` and the wave direction :math:`\theta_n`.

The frequency-domain Froude-Krylov forces are obtained from a linear potential flow-based solver such as *Nemoh* or *WAMIT*.

Weakly nonlinear model
**********************

Following the same method as for the weakly nonlinear hydrostatics, the computation of the weakly nonlinear Froude-Krylov loads involves the incident pressure integration over the body mesh at its real position. The free surface is represented by the plane :math:`z = 0`. Consequently, at every evaluation of the Froude-Krylov loads, the body mesh is clipped by the plane :math:`z = 0`. The expression of the Froude-Krylov force is:

.. math::
    \mathbf{f}_{FK}(t)= -\iint_{S_0} P_I \mathbf{n} dS

where

- :math:`P_I` is pressure of the incident wave field;
- :math:`\mathbf{n}` is normal vector, pointing outward the body surface;
- :math:`S_0` representes the wetted body surface delimited by the plane :math:`z = 0`.

The Froude-Krylov torque at the center of gravity of the body is expressed by:

.. math::
    \mathbf{\Gamma}_{FK}(t)= -\iint_{S_0} P_I (\mathbf{OM}-\mathbf{OG})\times\mathbf{n} dS

with

- :math:`\mathbf{OG}` the position of the center of gravity of the body;
- :math:`\mathbf{OM}` the position of the centroid of the panel.

Fully nonlinear model
*********************

In the fully nonlinear model for evaluating the Froude-Krylov loads, the mesh used for the pressure integration is delimited by the incident wave field :math:`z = \eta_I` and not the plane :math:`z = 0` as in the weakly nonlinear model. Consequently, the expression of the Froude-Krylov force and torque become:

.. math::
    \mathbf{f}_{FK}(t)= -\iint_{S_I} P_I \mathbf{n} dS

.. math::
    \mathbf{\Gamma}_{FK}(t)= -\iint_{S_I} P_I (\mathbf{OM}-\mathbf{OG})\times\mathbf{n} dS

where :math:`S_I` is the wetted body surface delimited by the incident wave field :math:`z = \eta_I`.

Diffraction force
-----------------

The diffraction force, given by the linear approximation, is:

.. math::
    \mathbf{f}_D(t) = \sum_m \sum_n \Im\left(A_{mn} \mathbf{f}_d(\omega_m,\theta_n) e^{j(k_m\bar{x}_n - \omega_m^e t + \Phi_{mn})}\right)

with

- :math:`\omega_m` the wave frequency;
- :math:`\theta_n` the wave direction;
- :math:`\bar{x}_n = x \cos(\theta_n) + y \sin(\theta_n)`;
- :math:`\omega_m^e = \omega_m - k_mU` is the encounter circular frequency, calculated using the steady forward speed;
- :math:`A_{mn}` the wave amplitude for the wave frequency :math:`\omega_m` and the wave direction :math:`\theta_n`;
- :math:`\mathbf{f}_D(\omega_m,\theta_n)` is the frequency-domain diffraction force component (with both the diffraction and Froude-Krylov components) for the wave frequency :math:`\omega` and the wave direction :math:`\theta_n`.

The frequency-domain diffraction forces are obtained from a linear potential flow-based solver such as *Nemoh* or *WAMIT*.

Exciting force
--------------

The excitation loads represent the combinaison of both the diffraction loads and the Froude-Krylov loads. 

.. math::
    \mathbf{f}_E(t) = \mathbf{f}_D(t) + \mathbf{f}_{FK}(t)

In the case of a linear Froude-Krylov model, the diffraction and Froude-Krylov loads are computed together:

.. math::
    \mathbf{f}_E(t) = \sum_m \sum_n \Im\left(A_{mn} \mathbf{f}_e(\omega_m,\theta_n) e^{j(k_m\bar{x}_n - \omega_m^e t + \Phi_{mn})}\right)

with

- :math:`\omega_m` the wave frequency;
- :math:`\theta_n` the wave direction;
- :math:`\bar{x}_n = x \cos(\theta_n) + y \sin(\theta_n)`;
- :math:`\omega_m^e = \omega_m - k_mU` is the encounter circular frequency, calculated using the steady forward speed;
- :math:`A_{mn}` the wave amplitude for the wave frequency :math:`\omega_m` and the wave direction :math:`\theta_n`;
- :math:`\mathbf{f}_e(\omega_m,\theta_n)` is the frequency-domain excitation force component (with both the diffraction and Froude-Krylov components) for the wave frequency :math:`\omega` and the wave direction :math:`\theta_n`.

The frequency-domain excitation forces are obtained from a linear potential flow-based solver such as *Nemoh* or *WAMIT*.

Radiation force
---------------

The generalized radiation force, given by the linear approximation, is:

.. math::
    \mathbf{f}_R(t) = -\mathbf{A}_{\infty} (U) \mathbf{\ddot{x}}(t) - \mathbf{B}_{\infty}(U) \mathbf{\dot{x}}(t)
                    - \int_0^t \mathbf{K}(t-\tau,U) \mathbf{\dot{x}}(\tau) d\tau

where

- :math:`\mathbf{x}` is the generalized position vector, in respect to the equilibrium frame;
- :math:`\mathbf{A}_{\infty} (U)` and :math:`\mathbf{B}_{\infty} (U)` are the infinite added mass and damping coefficient;
- :math:`\mathbf{K}` is the impulse response function.

The infinite added mass and damping coefficients are given by a linear potential flow based solver. The impulse response can be
computed from the frequency-domain damping coefficients.

.. math::
    \mathbf{K}(t) = \frac{2}{\pi} \int_0^{\infty} \mathbf{B}(\omega) \cos(\omega t) d\omega


Mean wave drift force
---------------------

The generalized mean wave drift force, given by the linear approximation, is:

.. math::
    \mathbf{f}_{WD} = \int_0^{2\pi} \int_0^{\infty} A(\omega,\theta) \mathbf{C}(\omega_e,\alpha) d\omega d\theta

where

- :math:`A_(\omega,\theta)` is the wave amplitude for the circular frequency :math:`\omega`, and wave direction :math:`\theta`;
- :math:`\omega_e` is the encounter circular frequency, which depends on :math:`(\omega,\theta)`;
- :math:`\alpha` is the relative angle between the wave direction and body heading;
- :math:`\mathbf{C}(\omega_e,\alpha)` are the polar wave drift coefficients, which depend on :math:`(\omega_e,\alpha)`.

