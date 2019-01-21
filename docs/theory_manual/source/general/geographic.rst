.. _geographic:

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
FRyDoM. The :any:`World reference frame origin <reference_frame>` can be set at a specified geographic coordinates, and
bodies position can be returned as geographic coordinates as well. Other FRyDoM methods using GeographicLib are
implemented.