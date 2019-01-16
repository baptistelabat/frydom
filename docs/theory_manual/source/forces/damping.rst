
Linear damping
______________

The linear damping force, applied on a body, can be expressed using generalized notations:

.. math::
     \mathbf{f}_{LD} = \mathbf{M}_{LD} \cdot \mathcal{V}

where :math:`\mathcal{V} = \begin{bmatrix} \mathbf{u} \\ \mathbf{\omega} \end{bmatrix}` is the generalized velocity of the
body.

The velocity of the body can be taken relatively to a fluid flow velocity (air or water).


Quadratic damping
_________________

The quadratic damping force is only applied on the translational velocity of a body, and therefor cannot be given using
generalized notations:

.. math::
    \mathbf{f}_{QD} = -\frac{1}{2} \rho_{fluid} \begin{bmatrix} C_x S_x |u_x| u_x \\C_y S_y |u_y| u_y \\C_z S_z |u_z| u_z \\ \end{bmatrix}

.. math::
    \mathbf{f}_i = -\frac{1}{2} \rho_{fluid} C_i S_i |u_i| u_i

where

- :math:`\rho_{fluid}` is the fluid density.
- :math:`C_i` are the damping coefficients,
- :math:`S_i` are the projected surfaces,
- :math:`\mathbf{u} = \begin{bmatrix}u_x & u_y & u_z \end{bmatrix}` is the body velocity. It can also be taken relatively to a fluid flow velocity, but be careful not to use a :any:`flow force<flow_force>` which might be redundant.

Manoeuvring damping
___________________

The manoeuvring damping force concerns surge, sway and yaw degrees of freedom. Each component of the damping force
is defined as the sum of non-linear terms depending in surge, sway and yaw velocities of the body.

.. math::
    \mathbf{f}_{MD} = \begin{bmatrix} \sum_k C_X(k) |u|^{m_k} |v|^{n_k} |w|^{p_k} sgn(u) sgn(v) sgn(w) \\ \sum_k C_Y(k) |u|^{m_k} |v|^{n_k} |w|^{p_k} sgn(u) sgn(v) sgn(w) \\ \sum_k C_N(k) |u|^{m_k} |v|^{n_k} |w|^{p_k} sgn(u) sgn(v) sgn(w) \end{bmatrix}

where

- :math:`C_X(k)`, :math:`C_Y(k)`, :math:`C_N(k)`  are the coefficients for the components respectively in surge, sway and yaw.
- :math:`u`, :math:`v`, :math:`w` are respectively the translational velocities in surge, sway and angular velocity in yaw.
