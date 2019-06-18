.. _radiation:

Radiation force
***************

Linear theory
=============

.. _impulse_response_and_convolution:

Following Cummin's method, the added mass and radiation damping loads on a body or group of bodies are computed, in the time
domain, using the following convolution integral equation:

.. math::
    \mathbf{f}_{R}(t) = - \mathbf{A}_{\infty} \mathbf{\ddot{x}}(t) - \int_0^t \mathbf{K}(t-\tau) \mathbf{\dot{x}}(\tau) d\tau

The impulse response function :math:`\mathbf{K}(t)` account for the past motion of the body, while the infinite added mass
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

Cutoff scaling function
-----------------------

The convolution involving the impulse response function requires to integrate from the start of the simulation, which leads
to an increasing cpu cost as the simulation progresses. However, since the impulse response function tends to decay to zero,
it is possible to cutoff the responses from the past motion of the body. Truncating the impulse response function may introduce
numerical errors and negative damping, with energy fed in the simulation. Scaling the impulse response function instead
ensure to limit the negative damping phenomena.

A cutoff scaling function can be applied when loading results from the linear potential flow based solver into the hydrodynamic
database, using the python scripts. The cutoff function is based on an exponential function:

.. math::
    c(t) = \exp\left(-\dfrac{3t}{T_c}\right)^2

where :math:`T_c` is the cutoff time.


Effect of constant forward speed on the radiation force
-------------------------------------------------------

When steady forward speed is considered, coupling effect between the yaw and pitch motion with the heave and sway force appears in the radiation model. In the case of slender body, the radiation convolution model presented in previous section can be modified to take into this effect [Rongere]_.

.. math::
    \mathbf{f}_R(t) = -\mathbf{A}(\infty, \mathbf{U}) \mathbf{\ddot{x}}(t) - \mathbf{B}(\infty, \mathbf{U})\mathbf{\dot{x}}(t) - \int_0^t \mathbf{K}(t-\tau, \mathbf{U}) \mathbf{\dot{x}}(\tau) d\tau

where :math:`\mathbf{B}(\infty, \mathbf{U})` is the linear damping term at infinity and the convolution term :math:`\mu(t)` :

.. math::
    \mu(t) = \int_0^t \mathbf{K}(t-\tau, \mathbf{U}) \mathbf{\dot{x}}(\tau) d\tau

The memory term can be decomposed into two functions :math:`\mu_0(t)` and :math:`\mu_U(t)` independent of :math:`\mathbf{U}` :

.. math::
    \mu(t) = \mu_0(t) + \mathbf{U} \mu_U(t)

with,

.. math::
    \mu_0(t) = \frac{2}{\pi} \int_0^{\infty} \mathbf{B}_0(\omega) \cos(\omega t) d\omega \\
    \mu_U(t) = \frac{2}{\pi} \int_0^{\infty} (\mathbf{A}_0(\infty) - \mathbf{A}_0(\omega)) \mathbf{L} \cos(\omega t) d\omega

where,

.. math::
    \mathbf{L} = \left( \begin{array}{cccc}
    0 & \ldots & 0 & 0 \\
    0 & \ldots & 0 & 1 \\
    0 & \ldots & -1 & 0 \\
    \vdots & \ddots & \vdots & \vdots \\
    0 & \ldots & 0 & 0 \\
    \end{array} \right)

The decomposition of the linear damping term takes the following forms:

.. math::
    \mathbf{B}_{\infty}(\mathbf{U}) &=& \lim\limits_{\omega \rightarrow +\infty} \mathbf{B}(\omega, \mathbf{U}) \\
        &=& \lim\limits_{\omega \rightarrow +\infty} \left( \mathbf{B}_0(\omega) + \mathbf{U} . \mathbf{B}_U(\omega) \right) \\
        &=& \lim\limits_{\omega \rightarrow +\infty} -\mathbf{U} \mathbf{A}_0(\omega) \mathbf{L} \\
        &=& -\mathbf{U} \mathbf{A}_0(\infty) \mathbf{L}


Reference:

.. [Rongere] F. Rongère, J.M. Kobus, A. Babarit, G. Delhommeau, "Comparative study of two methods to compute the radiation forces for a rowing application", 12eme Journées de l'Hydrodynamique, nantes, 17-19 novembre 2010



