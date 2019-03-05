.. body:

Body
====

Bodies are basic object within FRyDoM framework, on which can be applied forces, links, motors, cables, collisions box, etc.
Several frames of references can be used when referring to bodies.


Frames of reference
-------------------

.. toctree::
    :maxdepth: 1

    ../hydrodynamics/frames_of_reference


Inertia
-------

A body contains a reference frame and inertial quantities (mass :math:`m`, inertia matrix :math:`\mathbf{I}_G`, expressed
on a point :math:`G`), which can be grouped in an inertia tensor :math:`\mathbb{I}`.

.. math::
    \mathbb{I} = \Biggl \lbrace { m \atop \mathbf{I}_G } \Biggr \rbrace_G

The inertia matrix of a body depends on the choice of the reference point. The generalized Huygens theorem can be used to
express the inertia matrix on an other point:

.. math::
    \mathbf{I}_O = \mathbf{I}_G + \mathbf{I}(m,\mathbf{OG})

where, for :math:`\mathbf{OG} = \begin{bmatrix} a \\ b \\ c \end{bmatrix}`:

.. math::
    \mathbf{I}(m,\mathbf{OG}) = m \begin{bmatrix} b^2 + c^2 & -ab & -ac\\ -ab & a^2 + c^2 & -bc \\ -ac & -bc & a^2 + b^2 \end{bmatrix}

