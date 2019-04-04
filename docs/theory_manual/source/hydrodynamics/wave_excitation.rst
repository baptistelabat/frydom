.. _wave_excitation:

Wave excitation
===============

The wave excitation loads represent the combination of both the diffraction loads and the Froude-Krylov loads.

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

Froude-Krylov force
-------------------

The Froude-Krylov loads are due to the integration of the pressure of the incident wave field over the wetted body surface. Several models are available, following the same decomposition presented for the hydrostatic loads:

- a linear model;
- a weakly nonlinear model;
- a fully nonlinear model.

.. _linear_FroudeKrylov:

Linear model
************

The Froude-Krylov force, given by the linear approximation, is:

.. math::
    \mathbf{f}_{FK}(t) = \sum_m \sum_n \Im\left(A_{mn} \mathbf{f}_{fk}(\omega_m,\theta_n) e^{j(k_m\bar{x}_n - \omega_m^e t + \Phi_{mn})}\right)

with

- :math:`\omega_m` the wave frequency;
- :math:`\theta_n` the wave direction;
- :math:`\bar{x}_n = x \cos(\theta_n) + y \sin(\theta_n)`, with :math:`x` and :math:`y` the coordinates of the :any:`equilibrium frame <equilibrium_frame>`
- :math:`\omega_m^e = \omega_m - k_mU` is the encounter circular frequency, calculated using the steady forward speed;
- :math:`A_{mn}` the wave amplitude for the wave frequency :math:`\omega_m` and the wave direction :math:`\theta_n`;
- :math:`\mathbf{f}_{fk}(\omega_m,\theta_n)` is the frequency-domain Froude-Krylov force component for the wave frequency :math:`\omega` and the wave direction :math:`\theta_n`.

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

.. _linear_diffraction:

Linear Model
************

The diffraction force, given by the linear approximation, is:

.. math::
    \mathbf{f}_D(t) = \sum_m \sum_n \Im\left(A_{mn} \mathbf{f}_d(\omega_m,\theta_n) e^{j(k_m\bar{x}_n - \omega_m^e t + \Phi_{mn})}\right)

with

- :math:`\omega_m` the wave frequency;
- :math:`\theta_n` the wave direction;
- :math:`\bar{x}_n = x \cos(\theta_n) + y \sin(\theta_n)`, with :math:`x` and :math:`y` the coordinates of the :any:`equilibrium frame <equilibrium_frame>`
- :math:`\omega_m^e = \omega_m - k_mU` is the encounter circular frequency, calculated using the steady forward speed;
- :math:`A_{mn}` the wave amplitude for the wave frequency :math:`\omega_m` and the wave direction :math:`\theta_n`;
- :math:`\mathbf{f}_D(\omega_m,\theta_n)` is the frequency-domain diffraction force component for the wave frequency :math:`\omega` and the wave direction :math:`\theta_n`.

The frequency-domain diffraction forces are obtained from a linear potential flow-based solver such as *Nemoh* or *WAMIT*.

