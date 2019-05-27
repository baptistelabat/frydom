Demonstration of constraints (demo_Constraints.cpp) {#tutorial_demo_Constraints}
=====================================================

This demo features constraints between bodies, nodes, points, axis and planes. While kinematic links are used to model 
realistic links between bodies, constraints are more abstract and can be used at a conceptual level. 

Six different constraints are defined : 
- DistanceBetweenPoints, 
- DistanceToAxis, 
- PointOnPlane, 
- PlaneOnePlane,
- Perpendicular 
- Parallel. 

They are based either on points, axis and planes, and can be defined within FRyDoM using respectively the FrPoint, 
FrAxis, and FrPlane classes. Those are just plain abstractions of their geometric counterparts, based on FrNode. 
Since FrNodes belong to bodies, the constraints are applied in fine on the bodies. It is however easier to define these 
constraints using the aforementioned classes rather than the FrBody and FrNode directly.

The six constraints are illustrated below; for convenience, one of the two body is set fixed in the world reference frame
 and the other set free.

\include demo_Constraints.cpp
