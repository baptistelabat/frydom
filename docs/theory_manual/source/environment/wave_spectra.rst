
Wave spectra
~~~~~~~~~~~~

In construction

Jonswap Spectrum
----------------

This wave spectrum is an extension of the Pierson-Moskovitz wave spectrum, with an extra peak enhancement factor :math:`\gamma^r`.
The spreading function is given by [KIM20008]_ [MOLIN2002]_:

.. math::
    S_j(\omega) &=& \alpha H_s^2 \omega_p^4 \omega^{-5} \exp\left[-\frac{5}{4}\left(\frac{\omega_p}{\omega}\right)^4\right]\gamma^r \\
    \alpha &=& 0.3125\left(1 - 0.287\log(\gamma)\right)\\
    r &=& \exp \left[-\frac{(\omega-\omega_p)^2}{2\sigma^2\omega_p^2} \right] \\
    \sigma &=& |0.07, \omega < \omega_p\\
           & & |0.09, \omega > \omega_p


Default value for :math:`\gamma = 3.3` can be changed by the user, but has to be specified between 1 and 10.


Directional wave spectra : Cos2s
--------------------------------

This directional model, proposed by `Longuet-Higgins <ftp://ftp.mohid.com/Fortaleza_CD/Bibliografia/Waves/Directional%20Spectra.pdf>`_
[LonguetHiggins1963]_ is an extension of the cosine-squared model. The spreading function is given by:

.. math::
    D(f, \theta) = \left( \frac{2^{2s-1}}{\pi}\right)\left( \frac{\Gamma^2(s+1)}{\Gamma^2(2s+1)}\right) \cos^{2s}\
    \left(\frac{\theta - \theta_0}{2}\right)


where

- :math:`\Gamma =` the Gamma function,
- :math:`\theta_0 =` the mean wave direction,
- :math:`s =` the spreading parameter.

The spreading parameter is defined constant and can be set by the user.






References
~~~~~~~~~~

.. [LonguetHiggins1963] Longuet-Higgins, M.S., et al, *Observations of the Directional Spectrum of Sea Waves Using the Motions of a Floating Buoy*, Ocean Wave Spectra, Prentice-Hall, Inc., Englewood Cliffs, N. J., pp 111-13, 1963
.. [KIM20008]           Kim, C.H., *Nonlinear Waves and Offshore structures*, World Scientific Publishing Company, Vol.27, 2008
.. [MOLIN2002]          Molin, B., *Hydrodynamique des Structures Offshore*, Editions Technip, 2002