.. physics_item:

Physics items
=============

Every object with inner state variable(s) to be updated during the simulation can be considered a physics item. Bodies
and link are specialized physics items and are thus treated differently.

Examples of physics items
-------------------------

Catenary lines dynamic behavior is not solved by the constraint solver, but the :any:`quasi-static catenary equation <analytic_catenary>`.
As such, they can be considered as physics item. They provide to the bodies they link, the line tensions on each one.

In the same manner, the :any:`convolution of the impulse response function<impulse_response_and_convolution>` and the
past body motions is to be computed at each time step. A radiation convolution model has thus been defined as a physics
item, to manage this calculation.