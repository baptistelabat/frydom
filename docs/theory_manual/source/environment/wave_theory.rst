
Two linear wave field models are available, based on the Airy approximation : the regular and irregular wave models.
The Airy approximation is a linear approximation, i.e. for small amplitude waves.

Airy regular wave model
-----------------------

The regular wave is a single wave component defined by its amplitude :math:`A`, circular frequency :math:`\omega` and
direction :math:`\theta` . The wave height is measured from trough to crest, and is equal to twice the amplitude. The
wave period is :math:`T = 2\pi/\omega`.

The elevation of the regular wave, at a position :math:`(x,y)` and a time :math:`t` is given by:

.. math::
    \eta (x,y,t) = \Im(A \exp(jk\bar{x} - j\omega t)) = A \sin(k\bar{x} - \omega t)

where :math:`\bar{x} = x \cos(\theta) + y \sin(\theta)`.

The potential of the regular wave, at a position :math:`(x,y,z)` and a time :math:`t` is given by:

.. math::
    \phi (x,y,z,t) = \Im\left(-\frac{A j g}{\omega}   \exp(jk\bar{x} - j\omega t) \frac{\cosh(k(z+H)}{\cosh(kH)} \right)

The wave dispersion relation, :math:`\omega^2 = gk\tanh(kh)`, introduced in the previous equation, yields

.. math::
    \phi (x,y,z,t) = \Im\left(\frac{-A j \omega}{k}   \exp(jk\bar{x} - j\omega t) \frac{\cosh(k(z+H)}{\sinh(kH)} \right)

The last term is called the scaling factor, and can be subject to transformations (see :any:`kinematic stretching<wave_stretching>`)

.. math::
    E(z) = \frac{\cosh(k(z+H))}{\sinh(kH)}

Airy irregular wave model
-------------------------

The Airy irregular wave is a linear superposition of Airy regular waves of different wave circular frequency :math:`\omega_m` and
direction :math:`\theta_n`, with random wave phases :math:`\Phi_mn`.

.. math::
    \eta(x,y,t) = \sum_m \sum_n \Im(A_{mn} \exp(jk_m\bar{x}_n - j\omega_m t + \Phi_{mn}))

where :math:`\bar{x}_n = x \cos(\theta_n) + y \sin(\theta_n)`.

The wave frequency discretisation and direction discretisation are based on a constant discretisation : the wave components
are chosen to be equally spaced in frequency and direction.

The wave component amplitudes, :math:`A_{mn}`, are given by the directional spectrum (see :any:`wave spectra<wave_spectra>`)

.. math::
    A_{mn}^2 = 2S(\omega,\theta)\delta\omega\delta\theta

The velocity potential can thus be given by

.. math::
    \phi(x,y,z,t) = \sum_m \sum_n \Im(-j\frac{\omega A_{mn}}{k} E_m(z) \exp(jk_m\bar{x}_n - j\omega_m t + \Phi_{mn}))

where

.. math::
    E_m(z) = \frac{\cosh(k_m(z+H))}{\sinh(k_mH)}