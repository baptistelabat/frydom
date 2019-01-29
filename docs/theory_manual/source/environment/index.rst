Environment theory
******************

The environment model contains all information about the surrounding of the study.

Flow field model (wind and current)
===================================

In construction

Uniform flow field
------------------

Wind and current flow models are based on a uniform flow field : the flow velocity is constant anywhere in the domain.


Fluid physical properties (air and seawater)
============================================

In construction

The fluid physical properties modeled in FRyDoM are

- :math:`T` : temperature, in degree Celsius
- :math:`\rho` : density, in (kg/m³)
- :math:`\mu` : dynamic viscosity, in (Pa.s)
- :math:`\nu` : kinematic viscosity, in (m²/s)
- :math:`S_A` : salinity, in (g/kg)
- :math:`p` : pressure, in (MPa)

It is possible to get the seawater physical properties, for different temperatures and salinity, in the ITTC
recommended procedures [ITTC]_.

References
----------
.. [ITTC] ITTC Recommended Procedures : Fresh Water and Seawater Properties, 2011


Waves
=====

Wave theory
~~~~~~~~~~~

.. toctree::
    :maxdepth: 1

   wave_theory

Kinematic stretching
~~~~~~~~~~~~~~~~~~~~
.. toctree::
    :maxdepth: 1

   wave_stretching


Wave spectra
~~~~~~~~~~~~
.. toctree::
    :maxdepth: 1

   wave_spectra

Chart Datums
============

The chart datum is defined using the Lowest Astronomical Tide (LAT) : it is the lowest tide level which can be predicted
to occur under average meteorological conditions and under any combination of astronomical conditions. [LAT]_

Several heights can be given using this chart datum (see :any:`following figure <heights>`):

- :math:`B(x,y,)` : bathymetry, varying in space,
- :math:`H(t)` : tidal height, given by the tidal model,
- :math:`\eta(x,y,t)` : wave elevation, given by the :any:`wave field model<wave_theory>`,
- :math:`H(t)+\eta(x,y,t)` : free surface position, relatively to the LAT,
- :math:`H(t)-B(x,y,)` : ocean depth, relatively to the free surface.

.. _heights:
.. figure:: _static/heights.png
    :align: center
    :alt: chart datums

    Chart datum, wave elevation, tidal height and bathymetry definitions

References
----------

.. [LAT]  RESOLUTIONS of the INTERNATIONAL HYDROGRAPHIC ORGANIZATION Publication M-3 2nd Edition - 2010 Updated to December 2016. SECTION 2.2 – TIDES AND WATER LEVELS para 2 note (i)

Tidal model
===========
In construction

Sea bed model
=============
In construction


