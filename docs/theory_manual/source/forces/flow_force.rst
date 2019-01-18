.. _flow_force:

Current and wind drag loads due to translational relative velocity
------------------------------------------------------------------

Current and wind loads are computed using the standard OCIMF method [OCIMF]_. However, while the OCIMF method calculates the
drag loads on stationary bodies, with an absolute fluid velocity, FRyDoM consider instead the relative fluid velocity past the body.

The generalized flow force is then given by:

.. math::
    \mathcal{F}_{flow} = \frac{1}{2} \rho_{fluid} \begin{bmatrix}  C_X(\theta) A_X \\ C_Y(\theta) A_Y \\ 0\\0\\0\\ C_N(\theta) A_N  \end{bmatrix} |\mathbf{u}|^2

where

- :math:`\rho_{fluid}` is the fluid density (air or water),
- :math:`\mathbf{u}` is the relative velocity of the fluid, past the body.
- :math:`\theta` is the angle between the body heading and fluid flow velocity.
- :math:`(C_X, C_Y, C_N)` are the polar flow coefficients, respectively in surge sway and yaw, relatively to :math:`\theta`,
- :math:`A_X, A_Y, A_N` are the projected area of the body: above the waterline for the wind, below the waterline for the current,

The surge and sway forces are calculated in the body reference frame and projected on the horizontal plane of the world
reference frame afterwards, so that we do not get any vertical components. The yaw moment is also projected, so that
the moment acts about the vertical direction only.

.. todo:: add some figures to illustrate the flow velocity, etc.

The computations are done within FRyDoM in NWU and GOT conventions, however you can give polar coefficient in NED or NWU
and in GOTO or COMEFROM, convention.


References
----------
.. [OCIMF] Oil Companies International Marine Forum, 1994. Prediction of Wind and Current Loads on VLCCs, 2nd edition, Witherby & Co., London.