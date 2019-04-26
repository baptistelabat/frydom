Demonstration of Morison loads on a platform (demo_Morison.cpp) {#tutorial_demo_Morison}
=========================================================================

This demo features two cases of Morison loads applied on a structure. The first one is a basic horizontal cylinder.
Only one Morison element is needed to represent the Morison loads. The second structure is a semi-submersible platform
which requires the introduction of one element per underwater braces. All coefficients (added mass, drag and friction)
are given for the cylinder, but only drag coefficients are specified for the platform.

\include demo_Morison.cpp
