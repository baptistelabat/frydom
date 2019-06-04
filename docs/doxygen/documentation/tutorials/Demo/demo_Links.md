Demonstration of kinematic links (demo_Links.cpp) {#tutorial_demo_Links}
=====================================================

This demo features kinematic links between bodies :
- Fixed, 
- Revolute, 
- Cylindrical, 
- Prismatic,
- Spherical.

These are used to model realistic links; for more abstract constraints between bodies, see demo_Constraints (featuring
DistanceBetweenPoints, DistanceToAxis, PointOnPlane, PointOnline, PlaneOnPlane, Perpendicular and Parallel)

Kinematic links are based on FrNodes, belonging to bodies, to locate and orientate the kinematic link frames.

For convenience, one of the two body is set fixed in the world reference frame and the other set free.


\include demo_Links.cpp
