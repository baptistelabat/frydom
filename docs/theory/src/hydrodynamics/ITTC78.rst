.. _ITTC_resistance_force:

ITTC78 resistance force
-----------------------


The total resistance force of a ship without bilge keels, defined by the ITTC78 procedures [ITTC78]_, is

.. math::
    F = -\frac{1}{2} \rho S C_t V^2

where

- :math:`S` is the ship wetted area,
- :math:`V` is the ship velocity
- :math:`C_t` is the total resistance coefficient associated

The total resistance force is aligned with the heading of the ship and opposite to the vessel advance speed.

The total resistance coefficient is given by

.. math::
    C_t = (1+k) C_{f} + C_{r} + C_{a} + C_{aa} + C_{app}

where

- :math:`1+k` is the hull form coefficient,
- :math:`C_f` is the calculated frictional coefficient,
- :math:`C_r` is the residual coefficient,
- :math:`C_a` is the roughness allowance coefficient,
- :math:`C_{aa}` is the air resistance coefficient,
- :math:`C_{app}` is the appendage resistance coefficient,

The calculated frictional coefficient is given by

.. math::
    C_f = \frac{0.075}{(\log_{10}(Re) - 2)^2}

where :math:`Re = V L_{PP}/\nu` is the Reynolds number.

Except :math:`C_f`, all other coefficients must be given by the user. They are generally obtained through CFD calculations
or experiments. The residual coefficient can be obtained, for example, from model tests:

.. math::
    C_r = C_{t}^{meas} - (1+k)C_{f}^{meas}

where :math:`C_{t}^{meas}` and :math:`C_{f}^{meas}` are the total measured resistance coefficient and calculated model frictional
coefficient.

The form factor :math:`1+k` is usually determined from the resistance tests at low speed range or by Prohaska’s plot
of :math:`C_{f}^{meas}` against :math:`F_n^4`:

.. math::
    \frac{C_t}{C_f} = (1+k) + y \frac{F_n^4}{C_f}

where

- :math:`1+k` is the intercept on the :math:`C_t/C_f` axis,
- :math:`y` is the slope coefficient,

Reference:
~~~~~~~~~~

.. [ITTC78]     ITTC, *1978 ITTC Performance, Prediction Method*, ITTC Recommended Procedures, 1978
