Demonstration of cables applications (demo_Cable.cpp) {#tutorial_demo_Cable}
==========================

This demo presents cable features : catenary line and dynamic cable.

The catenary line is based on a quasi-static model, dedicated to simulate mooring line. This model is suitable for
lines with few or no dynamic, and not meant to work with too much traction. In this model, a uniform distributed
load is integrated on the line. Bending and torsion cannot be modeled. The quasi-static approach leads to fast
computations, compared to the dynamic cable.

The dynamic cable is based on Finite Element Analysis (FEA) solving, with an Euler Beam approximation. Bending and
torsion are integrated. More precise than the catenary line, it is however more time consuming. It is particularly
dedicated for cable with large motions and strains, heavy loads, violent dynamic simulations, etc.

Three demo are presented in this tutorial, with catenary lines and dynamic cables:

* a fixed line : both ends are constrained.
* a pendulum : a sphere balancing at the end of a line, with its other end fixed.
* a Newton pendulum : consists of a series of identically sized metal balls suspended in a metal frame so that they are
 just touching each other at rest. Each ball is attached to the frame by two lines of equal length angled away from 
 each other. This restricts the pendulums' movements to the same plane.

\include demo_Cable.cpp
