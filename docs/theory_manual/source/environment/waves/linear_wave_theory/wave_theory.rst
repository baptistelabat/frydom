.. _wave_theory:


Airy models
~~~~~~~~~~~

The Airy approximation is a linear approximation, i.e. for small amplitude waves. Two linear wave field models are available, based on the Airy approximation : the regular and irregular wave models.


Regular wave model
-----------------------

The regular wave is a single wave component defined by its amplitude :math:`A`, its circular frequency :math:`\omega` and its wave direction :math:`\theta` respect to the x-axis in NWU convention.

The elevation of the regular wave, at a position :math:`(x,y)` and a time :math:`t` is given by:

.. math::
    \eta (x,y,t) = A \sin (k \bar{x} - \omega t)

where :math:`\bar{x} = x \cos(\theta) + y \sin(\theta)`.

The wave height :math:`Hcc` is measured from trough to crest, and is equal to twice the amplitude. The
wave period is :math:`T = 2\pi/\omega`.

The Airy model is based on the incompressible and irrotationnal flow hypothesis, which implies that the velocity field can be derived as a gradient of a scalar function, the velocity potential :

.. math::
    \phi (x,y,z,t) = -\frac{A g}{\omega}\frac{\cosh(k(z+H)}{\cosh(kH)}\cos(\bar{x} - \omega t)

The wave dispersion relation, :math:`\omega^2 = gk\tanh(kh)`, introduced in the previous equation, yields

.. math::
    \phi (x,y,z,t) = -\frac{A \omega}{k}\frac{\cosh(k(z+H)}{\sinh(kH)}\cos(\bar{x} - \omega t)

One can note :math:`E(z)` the scaling factor which can be subject to transformations to limit inaccuracy of velocity prediction principally on the crest (see :any:`kinematic stretching<wave_stretching>`)

.. math::
    E(z) = \frac{\cosh(k(z+H))}{\sinh(kH)}

For infinite depth, the scaling factor :math:`E(z)` is defined by its asymptotic formula:

.. math::
    E(z) = exp( k z)

In finite depth, the orbital velocity of the flow is given by:

.. math::
    V_x = A \omega \cos \theta \frac{\cosh(k(z+H)}{\sinh(kH)}\sin(\bar{x} - \omega t) \\
    V_y = A \omega \sin \theta \frac{\cosh(k(z+H)}{\sinh(kH)}\sin(\bar{x} - \omega t) \\
    V_z = A \omega \frac{\sinh(k(z+H)}{\sinh(kH)} \cos(\bar{x} - \omega t)

The previous velocities definitions take the following form in infinite depth:

.. math::
    V_x = A \omega \cos(\theta) \exp(kz)\sin(\bar{x} - \omega t) \\
    V_y = A \omega \sin(\theta) \exp(kz)\sin(\bar{x} - \omega t) \\
    V_z = A \omega \exp(kz)\cos(\bar{x} - \omega t)


Airy irregular wave model
-------------------------

The Airy irregular wave is a linear superposition of Airy regular waves of different wave circular frequency :math:`\omega_m` and
direction :math:`\theta_n`, with random or specified wave phases :math:`\Phi_{mn}`.

.. math::
    \eta(x,y,t) = \sum_m \sum_n A_{mn} \sin(k_m\bar{x}_n - \omega_m t + \Phi_{mn})

where :math:`\bar{x}_n = x \cos(\theta_n) + y \sin(\theta_n)`.

The wave frequency discretisation and direction discretisation are based on a constant discretisation : the wave components
are chosen to be equally spaced in frequency and direction.

The wave component amplitudes, :math:`A_{mn}`, are given by the directional spectrum (see :any:`wave spectra<wave_spectra>`)

.. math::
    A_{mn}^2 = 2S(\omega,\theta)\delta\omega\delta\theta

The velocity potential can thus be given by

.. math::
    \phi(x,y,z,t) = \sum_m \sum_n -\frac{\omega_m A_{mn}}{k} E_m(z) \cos(k_m\bar{x}_n - \omega_m t + \Phi_{mn})

where

.. math::
    E_m(z) = \frac{\cosh(k_m(z+H))}{\sinh(k_mH)}


In infinite depth, the scaling factor is defined as follows:

.. math::
    E_m(z) = exp(k_m z)


In finite depth, the orbital velocity of the flow is given by:

.. math::
    V_x = \sum_m \sum_n A_{mn} \omega_m \cos \theta_n \frac{\cosh(k_m(z+H)}{\sinh(k_m H)}\sin(\bar{x_n} - \omega_m t) \\
    V_y = \sum_m \sum_n A_{mn} \omega_m \sin \theta_n \frac{\cosh(k_m(z+H)}{\sinh(k_m H)}\sin(\bar{x_n} - \omega_m t) \\
    V_z = \sum_m \sum_n A_{mn} \omega_m \frac{\sinh(k_m(z+H)}{\sinh(k_mH)} \cos(\bar{x_n} - \omega_m t)

The previous velocities definitions take the following form in infinite depth:

.. math::
    V_x = \sum_m \sum_n A_{mn} \omega_m \cos(\theta_n) \exp(k_m z)\sin(\bar{x_n} - \omega_m t) \\
    V_y = \sum_m \sum_n A_{mn} \omega_m \sin(\theta_n) \exp(k_m z)\sin(\bar{x_n} - \omega_m t) \\
    V_z = \sum_m \sum_n A_{mn} \omega_m \exp(k_m z)\cos(\bar{x_n} - \omega_m t)
