.. _coordinate_system:

Coordinate systems
******************

Cartesian coordinates systems
===================

Several reference frames are defined within FRyDoM. Frame definition, transformation and composition are detailed in \
:any:`frame theory <frame>`.

The main reference frame is the world reference frame, denoted  `W_{XYZ}`, which origin can be set at a specific geographic
position. Lots of reference frame are then defined, for bodies,
nodes on bodies (for locating kinematic links and cables), etc.

Coordinates systems are defined in FRyDoM as right-handed and positive rotations are clockwise. The following figure
illustrates the world reference frame and a body, with its coordinates system.

.. todo: .. images: _static/reference_frame.png

Two :any:`frame conventions <conventions>` are available in FRyDoM (NWU and NED). A frame convention must be specified
for setting/getting positions, velocities, directions, etc.

Environment related items (wind, current, waves, etc.) are generally given in the world reference frame. Position, velocity
and acceleration of a body can be specified in both world reference frame of local body coordinates system. Properties
of bodies depending of its orientation are usually given in the body local reference frame : drag, polar coefficients,
hydrodynamic database, etc. Only the inertia tensor can be given for any reference frame (world, local or other).

Reference frames can be drawn in Irrlicht through the user interface (type 'i' in the Irrlicht environment and check
'Draw COGs'. Increase the 'Symbol scale' if needed.)

Geographic coordinates system
=============================

The geographic coordinates system in FRyDoM is a coordinates system that locates any position on Earth, based on the
latitude, longitude and elevation coordinates. The latitude and longitude represent the horizontal position, while the
elevation represents the vertical position.

.. _fig_geographic_coordsys::
.. figure:: _static/geographic_coordsys.png
    :align: center
    :alt: Geographic Coordinate system

    Longitude lines are perpendicular to the Equator and latitude lines are parallel to the Equator.

Geographic coordinates in FRyDoM use the library GeographicLib, with the default Earth Geocentric (Geocentric::WGS84()).
For more information about the coordinate system definition, refer to \
`GeographicLib documentation <https://geographiclib.sourceforge.io/html/python/>`_

Conversion from/to cartesian coordinates
~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~

GeographicLib provides methods to convert geographic coordinates from/to cartesian coordinates, which are wrapped in
FRyDoM. The world reference frame origin can be set at a specified geographic coordinates, and
bodies position can be returned as geographic coordinates as well. Other FRyDoM methods using GeographicLib are
implemented.