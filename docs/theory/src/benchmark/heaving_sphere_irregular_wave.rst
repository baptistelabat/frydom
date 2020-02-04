.. _heaving_sphere_irregular_wave:

Heaving sphere in irregular waves
~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~

This comparison corresponds to the simulation of a sphere in heave motion submitted to irregular waves. The model is presented by the International Energy Agency (IEA) Offshore Energy System (OES) Task 10 [OES10]_ as a benchmark case for model validation and verification regrouping 25 organizations. The goal of this section is to compare the use of a linear and nonlinear approach for the computation of the hydrostatic and Froude-Krylov loads.

Description of the test case
----------------------------

The sphere considered in this simulation has a radius of :math:`5` \\(m\\) and a total mass of :math:`2,618 .10^5` \\(kg\\). At equilibrium, the center of the sphere is located on the mean water level and its center of gravity is located :math:`2` \\(m\\) below the water line. Main properties of the sphere are presented in the next table.

========================= ==================================
Parameters                Values
========================= ==================================
Radius                    :math:`5` \\(m\\)
Initial sphere location   (:math:`0`, :math:`0`, :math:`0`)
Center of gravity         (:math:`0`, :math:`0`, :math:`-2`)
Mass	                  :math:`261.8\times10^3` \\(kg\\)
Ixx                       :math:`1.690\times10^6` \\(kg.m^2\\) 
Iyy                       :math:`1.690\times10^6` \\(kg.m^2\\)
Izz                       :math:`2.606\times10^6` \\(kg.m^2\\)
Water detph               Inf
Water density             :math:`1000` \\(kg/m^3\\)
K33                       :math:`7.695\times10^5` \\(N/m\\)
K44                       :math:`5.126\times10^6` \\(N.m\\)
K55                       :math:`5.126\times10^6` \\(N.m\\)
========================= ==================================

The sphere is submitted to an irregular wave field propagating positive along the x-axis. A Jonswap wave spectrum is considered with a significant wave height (Hs) of :math:`0.5` \\(m\\), a spectral peak period (Tp) of :math:`4.4` \\(s\\) and a gamma factor (:math:`\gamma`) of :math:`1`.

Effects of a nonlinear hydrostatic and Froude-Krylov approach
-------------------------------------------------------------

The time series of the floating heaving sphere in irregular waves are now compared. Two models are considered:

 - a fully linear model;
 - a weakly nonlinear model: the hydrostatic and Froude-Krylov loads are computed with a fully nonlinear approach.

The two time series are plotted in :numref:`fig_sphere_irregular_wave_oes`. Due to the small steepness of the waves (:math:`0.26` %), the two models match perfectly, which validates their mutual implementation in irregular waves.

.. _fig_sphere_irregular_wave_oes:
.. figure:: _static/Comparison_Sphere_IW_Lin_Nonlin_hs_fk.png
    :align: center
    
    Comparison of the time series of a floating heaving sphere in an irregular wave field using a linear (blue) and fully nonlinear (orange) hydrostatic and Froude-Krylov model

References
----------

.. [OES10] F. Wendt, Y-H Yu, K. Ruehl, T. Bunnik, I. Touzon, B. W. Nam, J. S. Kim, K-H Kim, C. E. Janson, K-R. Jakobsen, S. Crowley, L. Vega, K. Rajagopalan, T. Mathai, D. Greaves, E. Ransley, P. Lamont-Kane, W. Sheng, R. Costello, B. Kennedy, S. Thomas, P. Heras, H. Bingham, A. Kurniawan, M. M. Kramer, D. Ogden, S. Girardin, A. Babarit, P.-Y. Wuillaume, D. Steinke, A. Roy, S. Betty, P. Shofield, J. Jansson and J. Hoffman, "International Energy Agency Ocean Energy Systems Task 10 Wave Energy Converter Modeling Verification and Validation", European Wave and Tidal Energy Conference, Cork, Ireland, 2017

.. [Nemoh] A. Babarit and G. Delhommeau, "Theoretical and numerical aspects of the open source BEM solver NEMOH", in Proc. of the 11th European Wave and Tidal Energy Conference", Nantes, France, 2015.
