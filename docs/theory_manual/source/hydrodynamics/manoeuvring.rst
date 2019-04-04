.. manoeuvring_loads:

Manoeuvring loads
-----------------

The manoeuvring damping force concerns surge, sway and yaw degrees of freedom. Each component of the damping force
is defined as the sum of non-linear terms depending in surge, sway and yaw velocities of the body.

.. math::
    \mathbf{f}_{MD} = \begin{bmatrix} \sum_k C_X(k) |u|^{m_k} |v|^{n_k} |w|^{p_k} sgn(u) sgn(v) sgn(w) \\ \sum_k C_Y(k) |u|^{m_k} |v|^{n_k} |w|^{p_k} sgn(u) sgn(v) sgn(w) \\ \sum_k C_N(k) |u|^{m_k} |v|^{n_k} |w|^{p_k} sgn(u) sgn(v) sgn(w) \end{bmatrix}

where

- :math:`C_X(k)`, :math:`C_Y(k)`, :math:`C_N(k)`  are the coefficients for the components respectively in surge, sway and yaw.
- :math:`u`, :math:`v`, :math:`w` are respectively the translational velocities in surge, sway and angular velocity in yaw.
