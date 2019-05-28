.. _constraints:

Constraints
===========

Constraints are specific kinematic links and are solved similarly. While kinematic links are used to model realistic
links between bodies, constraints are more abstract and can be used at a conceptual level.

They are based either on point, axis and plane geometric abstractions, which are relying, within FRyDoM, on nodes and
reference frames. Since nodes belong to bodies, the constraints are applied in fine on the bodies. It is however easier
to define these constraints using the geometric abstractions rather than the nodes and frames directly.

Six different constraints are implemented in FRyDoM (see next table)

.. |DistanceBetweenPoints| image:: _static/DistanceBetweenPoints.png
    :align: middle
.. |DistanceToAxis| image:: _static/DistanceToAxis.png
    :align: middle
.. |PointOnPlane| image:: _static/PointOnPlane.png
    :align: middle
.. |PlaneOnPlane| image:: _static/PlaneOnPlane.png
    :align: middle
.. |Perpendicular| image:: _static/Perpendicular.png
    :align: middle
.. |Parallel| image:: _static/Parallel.png
    :align: middle

=============================== =========================== ==============================
Name                            Symbol                      Constrained degrees of freedom
=============================== =========================== ==============================
Distance between points         |DistanceBetweenPoints|         1 translation, 0 rotation
Distance to axis                |DistanceToAxis|                2 translation, 0 rotation
Point on plane                  |PointOnPlane|                  1 translation, 0 rotation
Plane on plane                  |PlaneOnPlane|                  1 translation, 2 rotation
Perpendicular                   |Perpendicular|                 0 translation, 1 rotations
Parallel                        |Parallel|                      0 translation, 2 rotation
=============================== =========================== ==============================

Motions are constrained with respect to the :math:`x` , :math:`y` and :math:`z` of the first body. The constraints and
joints are thus applied along the axes of the first body.
