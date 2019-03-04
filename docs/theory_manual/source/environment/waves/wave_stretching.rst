.. wave_stretching:

Kinematic stretching
~~~~~~~~~~~~~~~~~~~~

The linear wave theory is based on the linearization of the free surface condition on the mean water level :math:`z=0`.
This means that the velocity potential evaluated above the mean water level can lead to non-realistic values, due to the
scaling factor :math:`E(z)`, in the velocity potential expression (see :any:`wave theory<wave_theory>`).

.. math::
    E(z) = \frac{\cosh(k(z+H))}{\sinh(kH)}

Stretching methods were developed to overcome this issue, by means of transformation techniques. They are introduced below
and integrated in FRyDoM.

Vertical stretching:
--------------------

The vertical stretching method simply assumes that the scaling factor above the mean water level is constant and equals to
its value at the mean water level. While :math:`E` is evaluated normally for :math:`z \leq 0`, the scaling factor becomes
for :math:`z > 0` :

.. math::
	E(t,\mathbf{x},z) = E(t,\mathbf{x},0)

This method results in an abrupt, non-physical, :math:`E` evolution when crossing the mean water level.

Extrapolation stretching:
-------------------------

To correct this non-physical evolution, the extrapolation method introduces a correction to the vertical method.
For :math:`z > 0` :

.. math::
	E(\mathbf{x},z,t) = E(\mathbf{x},0,t) + z \frac{\partial E}{\partial z}(\mathbf{x},0,t)

Wheeler stretching:
-------------------

This method stretches the vertical scale, in order to obtain at :math:`z=\eta`, the kinematic given by the linear theory
at :math:`z=0`. The corresponding transformation changes :math:`z` into :math:`z'` :

.. math::
    z' = H \frac{z-\eta}{H+\eta}

where :math:`H` is the water depth and :math:`\eta` is the instantaneous wave elevation.

The wheeler stretching is defined for : :math:`H < z < \eta`


Chakrabarti stretching:
-----------------------

In this method, proposed by Chakrabarti [Chakrabarti1971]_, the transformation is applied on the water depth. The term
scaling factor becomes:

.. math::
	E(z) = \frac{\cosh(k(z+h))}{\sinh(k(h+\eta))}

Delta-stretching:
-----------------

This method is a coupling between the Wheeler and the extrapolation stretching, proposed by Rodenbush and Forristall [Rodenbush1986]_.
While :math:`E(z)` is still replaced by :math:`E(z')`, the transformation :math:`z \longrightarrow z'` becomes

- for :math:`z > -H_{\Delta}` : :math:`z' = (z + H_{\Delta}) \frac{H_{\Delta} + \Delta.eta}{H_{\Delta} + \eta} - H_{\Delta}`
- for :math:`z < -H_{\Delta}` : :math:`z' = z`

:math:`\Delta` is a parameter taken between 0 and 1, and :math:`H_{\Delta}` is the water height on which the stretching is applied.

The Delta-stretching method can be downgraded to one of the two methods, in using specific values for :math:`H_{\Delta}`
and :math:`\Delta`:

- for :math:`H_{\Delta} = H` and :math:`\Delta = 0`, the method is equivalent to the Wheeler stretching.
- for :math:`H_{\Delta} = H` and :math:`\Delta = 1`, it is equivalent to the extrapolation stretching.

Rodenbush and Forristall recommend using :math:`\Delta = 0.3` and :math:`H_{\Delta} = H_S/2`. However Molin [Molin2002]_ suggests that
:math:`H_{\Delta} = H`, the water depth, is usually chosen.


References:
-----------

.. [Chakrabarti1971]   Chakrabarti, SK, *Discussion on "dynamics of single point mooring in deep water"*. J Waterwayss, Harbour and Coastal Eng Div ASCE, Vol 97, 588-590, 1971
.. [Rodenbush1986]     Rodenbusch, G, Forristall, GZ, *An empirical model for random wave kinematics near the free surface* Proc Offshore Technology Conf, Paper 5098, 1986
.. [Molin2002]         Molin, B., *Hydrodynamique des Structures Offshore*, Editions Technip, 2002