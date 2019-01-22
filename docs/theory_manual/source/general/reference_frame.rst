.. _reference_frames:

Reference frames
================

In construction

Several reference frames are defined within FRyDoM. Frame definition, transformation and composition are detailed in \
:any:`frame theory <frame>`.

The main reference frame is the world reference frame, denoted  `W_{XYZ}`, which origin can be set at a specific geographic
position (see :any:`Geographic-Cartesian conversion <geographic>`). Lots of reference frame are then defined, for bodies,
nodes on bodies (for locating kinematic links and cables), etc.

Coordinates systems are defined in FRyDoM as right-handed and positive rotations are clockwise. The following figure
illustrates the world reference frame and a body, with its coordinates system.

.. todo: .. images: _static/reference_frame.png

Two :any:`frame conventiond <convention>` are available in FRyDoM (NWU and NED). The frame convention is to be specified
for setting/getting positions, velocities, directions, etc.
Environment related items (wind, current, waves, etc.) are generally given in the world reference frame. Position, velocity
and acceleration of a body can be specified in both world reference frame of local body coordinates system. Properties
of bodies depending of its orientation are usually given in the body local reference frame : drag, polar coefficients,
hydrodynamic database, etc. Only the inertia tensor can be given for any reference frame (world, local or other).

Reference frames can be drawn in Irrlicht through the user interface (type 'i' in the Irrlicht environment and check
'Draw COGs'. Increase the 'Symbol scale' if needed.)