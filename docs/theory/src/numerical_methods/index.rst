.. _numerical_methods:

Numerical methods
*****************

.. _time_steppers:

Time steppers
=============

Several time steppers are available within chrono physics engine:

- Euler explicit
- Euler implicit
- Euler implicit linearized
- Runge-Kutta 45
- Trapezoidal
- Newmark
- HHT

Some of these time steppers are dedicated to smooth dynamics: for classical multibody dynamics with rigid and flexible
bodies connected through links, finite element analysis or fluid solid interaction problems.

Others are more efficient for non-smooth dynamics : ie with friction and contact between bodies.

For more information on these time steppers, please refer to `this page <http://www.projectchrono.org/assets/slides_3_0_0/3_Contact/3_Chrono_Solvers.pdf>`_.

.. _constraint_solvers:

Constraint Solvers
==================

*In construction*
