Defining non linear hydrostatics models (demo_Hydrostatics.cpp) {#tutorial_demo_Hydrostatics}
===============================================================

This demo presents two non linear hydrostatic models. In both, the hydrostatic force is computed by integrating
the hydrostatic pressure over the wetted surface. In the weakly non linear hydrostatic model, the wetted surface
is defined by the exact position of the body, clipped by the mean free surface plane (z = 0). In the non linear
hydrostatic model, the wetted surface is still defined by the exact position of the body, but delimited by the
incident wave field.

This demo considers a cylinder with a mesh of 2900 elements, over which both models can be applied. (select the
one you want to test).

\include demo_NonLinearHydrostatics.cpp
