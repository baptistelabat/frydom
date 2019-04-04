.. _body:

Bodies
======

Bodies are basic object within FRyDoM framework, on which can be applied body components :

- forces,
- links and motors,
- cables,
- collisions box, etc.


Frames of reference
-------------------

..    définition des repères utilisés pour un corps.


Several frames of references can be used when referring to bodies.


.. TODO
.. _fig_reference_frames:
.. figure:: _static/todo.png
    :align: center
    :alt: Frames of reference on a body

    Representation of the body and COG reference frames


Body reference frame
~~~~~~~~~~~~~~~~~~~~

The body reference plane locates the body relatively to the :any:`world reference frame <frame>`. Its origin is defined arbitrary by
the user: it can be located at the bow on the keel, at the center of gravity, etc. Its orientation gives the direction of
the degrees of freedom:

- surge and roll are respectively the translation and rotation related to the x axis of the reference frame,
- sway and pitch are respectively the translation and rotation related to the y axis of the reference frame,
- heave and yaw are respectively the translation and rotation related to the z axis of the reference frame.

All body components (nodes, links, forces) are then set up, relatively to this reference frame.

The accessors and mutators  (GetPointPositionInBody, SetGeneralizedVelocityInBody, TranslateInBody, etc.) which name
contain "InBody", refer to this body reference frame.

The body reference frame is the instantaneous reference frame of the body, which means it follows the body in its motions
and rotations.

COG reference frame
~~~~~~~~~~~~~~~~~~~

The COG reference frame is usually based on a similar orientation as the body reference frame, but its origin is located
at the body center of gravity (COG). Most forces are applied on the COG reference frame.



Mass and inertia
----------------

All bodies must contain mass and inertial quantities (mass :math:`m`, inertia matrix :math:`\mathbf{I}_G`, expressed
on a point :math:`G`); all three can be grouped in an inertia tensor :math:`\mathbb{I}` :

.. math::
    \mathbb{I} = \Biggl \lbrace { m \atop \mathbf{I}_G } \Biggr \rbrace_G

The inertia matrix of a body depends on the choice of the reference point. The generalized Huygens theorem can be used to
express the inertia matrix on an other point:

.. math::
    \mathbf{I}_O = \mathbf{I}_G + \mathbf{I}(m,\mathbf{OG})

where, for :math:`\mathbf{OG} = \begin{bmatrix} a \\ b \\ c \end{bmatrix}`:

.. math::
    \mathbf{I}(m,\mathbf{OG}) = m \begin{bmatrix} b^2 + c^2 & -ab & -ac\\ -ab & a^2 + c^2 & -bc \\ -ac & -bc & a^2 + b^2 \end{bmatrix}



Rotations
---------

FRyDoM internal body dynamics is based on the use of quaternion to avoid gimbal lock. However FRyDoM allows users to work
with different rotation formats, including quaternions and Euler angles.

The direction of the rotation (positive or negative) is given by the direct definition of the frames of reference.
A :math:`\pi /2`  positive rotation around z axis transforms the x axis into y axis.

Unit quaternions
~~~~~~~~~~~~~~~~


.. définition des unit quaternions

Euler angles
~~~~~~~~~~~~

The Euler angles are a combination of three rotation angles used to represent any rotation or the orientation of a frame
relatively to another. These angles are elemental rotations around the axis of a frame of reference. They are usually
called :math:`(\phi,\theta,\psi)` but different sequences exists.

Cardan sequence
***************

.. Euler Angle Sequence (1,2,3)

The Cardan angles are denoted as yaw, pitch and roll, and correspond to the Euler sequence (1,2,3).
It consists of three consecutive rotations, as shown in  :any:`this figure <fig_Cardan_angles>` :

- first rotation of an angle :math:`\psi` around the z axis,
- second rotation of an angle :math:`\theta` around the y''' axis,
- third rotation of an angle :math:`phi` around the x'' axis.

The body reference frame x axis is defined toward the front of the body for both NED and NWU :any:`frame convention <conventions>`.
However in NED the y axis is along the starboard while the z axis downward. In NWU, the y axis point to port and the z upward.
The direct implications on rotation is that a positive change in :math:`\theta` corresponds to pitching downward in NWU,
and upward in NED.


.. _fig_Cardan_angles:
.. figure:: _static/Cardan_angles.png
    :align: center
    :alt: Cardan angles

    Representation of the Cardan angles, from Diebel [DIEBEL]_

For more information on rotation matrix, and function thats pas Cardan angles to their corresponding unit quaternion,
please refer to Diebel [DIEBEL]_.


References

.. [DIEBEL] Diebel, J., Representing Attitude: Euler Angles, Unit Quaternions, and Rotation Vectors, 2006, Standford University, https://www.astro.rug.nl/software/kapteyn/_downloads/attitude.pdf


Connections to components
-------------------------

Bodies can be connected using :any:`links <links>`, :any:`motors <motors>` or :any:`cables <cables>`. To position bodies
and their components, FRyDoM relies on nodes, which posses their own reference frame, and defined relatively to the body frame.


Nodes
~~~~~

..
    Node descriptions

Degrees of freedom
~~~~~~~~~~~~~~~~~

Body's degrees of frydom can be restricted or set free.

..
    a décrire : world body, DOF Mask, etc...


