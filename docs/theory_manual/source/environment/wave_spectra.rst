
Wave spectra
~~~~~~~~~~~~

In construction

Pierson-Moskowitz spectrum
--------------------------

The Pierson-Moskowitz spectrum :math:`S_{PM}(\omega)` is given by [DNV]_:

.. math::
    S_{PM}(\omega) = \frac{5}{16} \cdot H_S^2 \omega_p^4 \cdot \omega^{-5} \exp\left[-\frac{5}{4}\left(\frac{\omega}{\omega_p}\right)^{-4}\right]

where :math:`\omega_p = 2\pi / T_p` is the angular spectral peak frequency.

JONSWAP spectrum
----------------

The JONSWAP wave spectrum is an extension of the Pierson-Moskovitz wave spectrum, for a developing sea state in a fetch limited situation,
with an extra peak enhancement factor :math:`\gamma^r`. The spreading function is given by [DNV]_ [KIM20008]_ [MOLIN2002]_:

.. math::
    S_j(\omega) = \alpha_{\gamma} S_{PM}(\omega)  \gamma^{\exp \left[-\frac{(\omega-\omega_p)^2}{2\sigma^2\omega_p^2} \right]}

where

- :math:`S_{PM}(\omega)` is the Pierson-Moskowitz spectrum,
- :math:`\gamma` is the non-dimensional peak shape parameter,
- :math:`\sigma` is the spectral width parameter,
.. math::
    \sigma &=& 0.07, \omega \leq \omega_p\\
    \sigma &=& 0.09, \omega > \omega_p
- :math:`\alpha_{\gamma}= 1 - 0.287\log(\gamma)` is a normalizing factor.

The JONSWAP spectrum is expected to be a reasonable model for :math:`3.6<T_p/H_S<5`.
Default value for :math:`\gamma = 3.3` can be changed by the user, but has to be specified between 1 and 10.


Directional wave spectra
------------------------

Directional short-crested wave spectra :math:`S(\omega,\theta)` is expressed in terms of uni-directional wave spectra,

.. math::
    S(\omega,\theta) = S(\omega)D(\theta)

where :math:`D(\theta)` is a directional function, :math:`\theta` is the angle between the direction of elementary wave trains
and the main wave direction of the short crested wave system. The directional function fulfils the requirement:

.. math::
   \int  D(\theta) d\theta = 1

Cos2s directional function
__________________________

This directional model, proposed by `Longuet-Higgins <ftp://ftp.mohid.com/Fortaleza_CD/Bibliografia/Waves/Directional%20Spectra.pdf>`_
[LonguetHiggins1963]_ is an extension of the cosine-squared model. The spreading function is given by:

.. math::
    D(\theta) = \frac{2^{2s-1}}{\pi} \frac{\Gamma^2(s+1)}{\Gamma^2(2s+1)} \cos^{2s} \left(\frac{\theta - \theta_0}{2}\right)

where

- :math:`\Gamma` is the Gamma function,
- :math:`\theta_0` is the mean wave direction,
- :math:`s` is the spreading parameter.

The spreading parameter is defined constant and can be set by the user.






References
~~~~~~~~~~

.. [LonguetHiggins1963] Longuet-Higgins, M.S., et al, *Observations of the Directional Spectrum of Sea Waves Using the Motions of a Floating Buoy*, Ocean Wave Spectra, Prentice-Hall, Inc., Englewood Cliffs, N. J., pp 111-13, 1963
.. [KIM20008]           Kim, C.H., *Nonlinear Waves and Offshore structures*, World Scientific Publishing Company, Vol.27, 2008
.. [MOLIN2002]          Molin, B., *Hydrodynamique des Structures Offshore*, Editions Technip, 2002
.. [DNV]                VERITAS, Det Norske. Modelling and analysis of marine operations. Offshore Standard, 2011.