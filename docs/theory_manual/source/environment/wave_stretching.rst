.. _wave_stretching:

With linear wave theory, the linearization of the free surface condition around the mean water level :math:`z=0` can produce some irrealistic flow velocity in the wave crest for irregular wave profiles. This is of particular importance when drag force is evaluated from this velocity since it evolves with the square of the velocity. To limit this effect, various stretching technique have been proposed. FRyDoM integrates some of the most used among them.

Vertical stretching:
--------------------

With the vertical stretching it is assumed that the flow velocity equal the velocity given by the linear wave theory when :math:`z<0` whereas when :math:`z>0` the velocity equal the velocity at the mean water level :math:`u(z=0)`.

.. math::
	u(z) = u(t,x,y,z) if z < 0
	     = u(t,x,y,z=0) if z >= 0

Extrapolation stretching:
-------------------------

To avoid abrupt velocity modification in :math:`z=0` the extrapolation stretching can be used which assumed that the velocity take the following form for :math:`z>0`

.. math::
	u(x,z,t) = u(x,0,t) + k \frac{\partial u}{\partial z}(x,0,t)

Wheeler stretching:
-------------------

An other weel known approach is the wheeler stretching. In this approach a modification of the z-coordinate is used to take into account the variation of the water column due to the variation of the free surface.

.. math::
	z' = h \frac{(h+z)}{(h+\eta)}-h

where :math:`h` is the water depth and :math:`\eta` the free surface elevation.

The flow velocity is evalued from :math:`u(z')`.

Chakrabarti stretching:
-----------------------

An other approach is proposed by Chakrabarti with the modification of the water depth:

.. math::
	u(x,z,t) = \sum_i A_i * \omega_i \frac{\ch k_i (z+h)}{\sh k_i(h + \eta)} cos(k_i x - \omega_i t + \tetha_i)

Delta-stretching:
-----------------

Presented by Rodenbush and Forristall [1986] the delta-stretching present an intermediate profile in between the extrapolation stretching and Wheeler model.

.. math::
	z' = (z+h_{\Delta}) \frac{h_{\Delta} + \Delta \eta}{h_{\Delta} + \eta} - h_{\Delta}

where :math:`\Delta`is a parameter between 0 and 1 and :math:`h_{\Delta}`is the water depth to which the stretching is applied.