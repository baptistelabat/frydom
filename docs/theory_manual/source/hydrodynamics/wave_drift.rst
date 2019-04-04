.. _wave_drift:

Mean wave drift force
---------------------

The generalized mean wave drift force, given by the linear approximation, is:

.. math::
    \mathbf{f}_{WD} = \int_0^{2\pi} \int_0^{\infty} A(\omega,\theta) \mathbf{C}(\omega_e,\alpha) d\omega d\theta

where

- :math:`A_(\omega,\theta)` is the wave amplitude for the circular frequency :math:`\omega`, and wave direction :math:`\theta`;
- :math:`\omega_e` is the encounter circular frequency, which depends on :math:`(\omega,\theta)`;
- :math:`\alpha` is the relative angle between the wave direction and body heading, with respect to the :any:`equilibrium frame <equilibrium_frame>`;
- :math:`\mathbf{C}(\omega_e,\alpha)` are the polar wave drift coefficients, which depend on :math:`(\omega_e,\alpha)`.

