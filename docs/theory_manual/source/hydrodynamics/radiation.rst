.. _radiation:

Radiation force
***************

Linear theory
=============

The generalized radiation force, given by the linear approximation, is:

.. math::
    \mathbf{f}_R(t) = -\mathbf{A}_{\infty} \mathbf{\ddot{x}}(t) - \int_0^t \mathbf{K}(t-\tau) \mathbf{\dot{x}}(\tau) d\tau

where

- :math:`\mathbf{x}` is the generalized position vector, with respect to the :any:`equilibrium frame <equilibrium_frame>`;
- :math:`\mathbf{A}_{\infty}` and :math:`\mathbf{B}_{\infty}` are the infinite added mass and damping coefficient,
- :math:`\mathbf{K(t)}` is the impulse response function.

The infinite added mass and damping coefficients are given by a linear potential flow based solver. The impulse response can be
computed from the frequency-domain damping coefficients.

.. math::
    \mathbf{K}(t) = \frac{2}{\pi} \int_0^{\infty} \mathbf{B}(\omega) \cos(\omega t) d\omega


.. _impulse_response_and_convolution:

Impulse response and convolution
================================

Following Cummin's method, the added mass and radiation damping loads on a body or group of bodies are computed, in the time
domain, using the following convolution integral equation:

.. math::
    \mathbf{f}_{R}(t) = \mathbf{A}_{\infty} \mathbf{\ddot{x}}(t) + \int_0^t \mathbf{K}(t-\tau) \mathbf{\dot{x}}(\tau) d\tau

The impulse response function :math`\mathbf{K}` account for the past motion of the body, while the infinite added mass
:math:`\mathbf{A}_{\infty}` gives the body's instantaneous response to acceleration. All these terms are to be computed using the
frequency-dependant added mass and radiation damping coefficients.

Impulse response function (IRF)
-------------------------------

The infinite added mass and damping coefficients are given by a linear potential flow based solver. The impulse response can be
computed from the frequency-domain damping coefficients.

.. math::
    \mathbf{K}(t) = c(t) \frac{2}{\pi} \int_0^{\infty} \mathbf{B}(\omega) \cos(\omega t) d\omega

where

- :math:`\mathbf{B}(\omega)` is the frequency-dependant damping matrix, at circular frequency :math:`\omega`,
- :math:`c(t)` is a cutoff scaling function

Impulse response function cutoff
~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~

The convolution involving the impulse response function requires to integrate from the start of the simulation, which leads
to an increasing cpu cost as the simulation progresses. However, since the impulse response function tends to decay to zero,
it is possible to cutoff the responses from the past motion of the body. Truncating the impulse response function may introduce
numerical errors and negative damping, with energy fed in the simulation. Scaling the impulse response function instead
ensure to limit the negative damping phenomena.

A cutoff scaling function can be applied when loading results from the linear potential flow based solver into the hydrodynamic
database, using the python scripts. The cutoff function is based on an exponential function:

.. math::
    c(t) = exp (-\dfrac{3t}{T_c})

where :math:`T_c` is the cutoff time.


Infinite-frequency added mass
-----------------------------

In construction


Infinite-frequency radiation damping
------------------------------------

In construction


