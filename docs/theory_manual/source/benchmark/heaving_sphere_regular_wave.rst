.. _heaving_sphere_regular_wave:

Heaving sphere in regular waves
~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~

This benchmark corresponds to the simulation of a sphere in heave motion submitted to regular waves. This simulation is presented by the International Energy Agency (IEA) Offshore Energy System (OES) Task 10 [OES10]_ as a benchmark case for model validation and verification regrouping 25 organizations. Description of this test case and results obtained by FRyDoM are sumarized in the following.

Description of the test case
----------------------------

The sphere considered in this simulation has a radius of :math:`5m` and a total mass of :math:`2,618 .10^5` kg. At equilibrium, the center of the sphere is located on the mean water level and its center of gravity is located :math:`2m` below the water line. Main properties of the sphere are presented in the next table.

========================= =======================
Parameters                Values
========================= =======================
Radius                    5 m
Initial sphere location   (0, 0, 0)
Center of gravity         (0, 0, -2)
Mass	                  :math:`261,8.10^3` kg
Ixx                       :math:`1,690.10^6`
Iyy                       :math:`1,690.10^6`
Izz                       :math:`2,606.10^6`
Water detph               Inf
Water density             :math:`1000 kg/m^3`
K33                       :math:`7,695.10^5`
K44                       :math:`5,126.10^6`
K55                       :math:`5,126.10^6`
========================= =======================

The sphere is submitted to regular wave propagating positive along the x-axis. The wave periods considered in this test case varies from 3s to 11s with a steepness equal to 0.2%.


Results
-------

The response amplitude operator (RAO) in heave motion obtained from FRyDoM is presented in Fig. <fig_sphere_regular_wave>_. Very good agreement with the results obtained by Nemoh [Nemoh]_ can be observed.

.. _fig_shere_regular_wave:
.. figure:: _static/sphere_regular_s0_002.png
    :align: center
    :alt: Heave RAO


For illustrative prupose, the restuls obtained from the other numerical models presented in [OES10]_ for the same test case are shown in next figure

.. _fig_shere_regular_wave_oes:
.. figure:: _static/sphere_regular_s0_002_oes.png
    :align: center
    :alt: Heave RAO


References
----------

.. [OES10] F. Wendt, Y-H Yu, K. Ruehl, T. Bunnik, I. Touzon, B. W. Nam, J. S. Kim, K-H Kim, C. E. Janson, K-R. Jakobsen, S. Crowley, L. Vega, K. Rajagopalan, T. Mathai, D. Greaves, E. Ransley, P. Lamont-Kane, W. Sheng, R. Costello, B. Kennedy, S. Thomas, P. Heras, H. Bingham, A. Kurniawan, M. M. Kramer, D. Ogden, S. Girardin, A. Babarit, P-Y. Wuillaume, D. Steinke, A. Roy, S. Betty, P. Shofield, J. Jansson and J. Hoffman, "International Energy Agency Ocean Energy Systems Task 10 Wave Energy Converter Modeleing Verification and Validation", European Wave and Tidal Energy Conference, Cork, Ireland, 2017

.. [Nemoh] A. Babarit and G. Delhommeau, "Theoretical and numerical aspects of the open source BEM solver NEMOH", in Proc. of the 11th European Wave and Tidal Energy Conference", Nantes, France, 2015.
