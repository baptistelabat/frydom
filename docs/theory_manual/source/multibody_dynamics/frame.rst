.. _frame:


Frames
======

A frame contains an origin and a system of coordinates, which consists of a set of 3 axes in 3D. It can then be represented
as a tensor object:

.. math::
    \mathcal{F} = \Biggl \lbrace { \mathbf{P} \atop \begin{bmatrix} \mathbf{e}_1 & \mathbf{e}_2 & \mathbf{e}_3 \end{bmatrix} } \Biggr \rbrace

The position of the origin, :math:`\mathbf{P}`, and the orientation of the 3 axes,
:math:`\begin{bmatrix} \mathbf{e}_1 & \mathbf{e}_2 & \mathbf{e}_3 \end{bmatrix}`, are expressed in an other frame.
So a frame can also be considered as a frame transformation from a parent frame, with a translation vector and a rotation
matrix. Note that a frame has no reference to its parent frame.


.. todo: .. images: _static/frame_definition.png


The world reference frame is the arbitrary frame of reference of the simulation. Its geographic origin can be set up,
however it's orientation is mostly defined : the x axis is always pointing North. The 2 available :any:`frame conventions <convention>`
are indeed North-West-Up or North-East-Down.



.. In order to have a fully explicit frame notation, we need to specify the parent frame. We then chose the following notation: :math:`^j\mathbb{F}_i` corresponds to the frame :math:`i`, expressed in the frame :math:`j`. It can also represent the frame transformation from frame :math:`i` to frame :math:`j`. In the same manner, :math:`^iv_j` is the velocity of frame :math:`i`, expressed in :math:`j`. It can be expressed in


