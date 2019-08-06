Creating and setting mooring lines (demo_MooringLines.cpp) {#tutorial_demo_MooringLines}
==========================================================

This demo presents the definition of dynamic cable. A distinction is to be made between cables taut or not at initialisation.

In the case of non taut cable, the dynamic cable position is initialized using a non elastic catenary model.
If the elasticity of the cable is not too large, the dynamic cable position is close to its static equilibrium.

For taut cable, the dynamic cable is initialized unstrained and then strained either during a static analysis
or the dynamic simulation. This means that the cable ending node is not located at its specified location.

The hinges (joints) to connect the cable to bodies can be defined as completely constrained (no degrees of freedom)
or spherical (degree of freedom in rotation not constrained). By default, they are defined as spherical.

Be careful on the definition of the frame nodes, on the bodies, in the case of constrained hinges. The x axis of
the frame node must corresponds to the direction of the cable you want to get at this cable end.

\include demo_MooringLines.cpp
