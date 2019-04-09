.. _other_damping:

Damping force
=============

Linear damping
--------------

The linear damping force, applied on a body, can be expressed using generalized notations:

.. math::
     \mathbf{f}_{LD} = \mathbf{M}_{LD} \mathcal{V}

[CC: la notation de vitesse généralisée utilisée n'est pas la même que dans le reste du document (\mathbf{x}). Uniformiser les notations]

where :math:`\mathcal{V} = \begin{bmatrix} \mathbf{u} \\ \mathbf{\omega} \end{bmatrix}` is the generalized velocity of the
body and :math:`\mathbf{M}_{LD}` is the damping matrix.

The velocity of the body can be taken relatively to a fluid flow velocity (air or water).


Quadratic damping
-----------------

The quadratic damping force is only applied on the translational velocity of a body, and therefor cannot be given using
generalized notations:

.. math::
    \mathbf{f}_{QD} = -\frac{1}{2} \rho_{fluid} \begin{bmatrix} C_x S_x |u_x| u_x \\C_y S_y |u_y| u_y \\C_z S_z |u_z| u_z \\ \end{bmatrix}

.. math::
    \mathbf{f}_{QDi} = -\frac{1}{2} \rho_{fluid} C_i S_i |u_i| u_i

[CC: ne garder qu'une seule notation]

where

- :math:`\rho_{fluid}` is the fluid density.
- :math:`C_i` are the damping coefficients,
- :math:`S_i` are the projected surfaces, [CC: préciser par rapport à quelle position (equilibre, instantannée ?)]
- :math:`\mathbf{u} = \begin{bmatrix}u_x & u_y & u_z \end{bmatrix}` is the body velocity. It can also be taken relatively to a fluid flow velocity, but be careful not to use a :any:`flow force<flow_force>` which might be redundant.
