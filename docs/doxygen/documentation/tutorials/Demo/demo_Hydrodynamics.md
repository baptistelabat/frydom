Demonstration of hydrodynamic loads on a platform (demo_Hydrodynamic.cpp) {#tutorial_demo_Hydrodynamic}
=========================================================================

This demo features linear hydrodynamic loads on a semi submersible platform. Based on the linear potential flow
theory, the hydrodynamic loads consists in hydrostatic, wave excitation (wave diffraction and Froude-Krylov) and radiation
damping loads. Wave drift load can also be added, but needs additional parameters.
The linear potential flow theory assumptions (small amplitude of motions of the body and small wave curvature)
requires the introduction of a body equilibrium frame, for expressing the different loads.

Wind and current loads are also taken into account, based on polar coefficients.

\include demo_Hydrodynamic.cpp
