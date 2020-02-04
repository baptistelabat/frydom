.. _hydrostatic_equilibrium:


Hydrostatic equilibrium
=======================

Modeling a floating body with a specific loading requires first to determine its hydrostatic equilibrium. A dedicated
solver is included in FRyDoM, based on the developments for the nonlinear hydrostatic model.

Linear iterative solver
-----------------------

The workflow is very similar to what already exists in Meshmagick. The hydrostatic equilibrium solver thus require the
mesh of the body, correctly located relatively to the body reference frame, and a loading (mass and COG position).

A Newton-Raphson solver is used to find iteratively the hydrostatic equilibrium, according to the well-known equation

.. math::
    \mathbb{K} \begin{bmatrix} z\\ \phi \\ \theta \end{bmatrix} =
    \begin{bmatrix} \rho g \nabla - m g\\
        \rho g \nabla \mathbf{B}.\mathbf{y} - m g \mathbf{G}.\mathbf{y} \\
        \rho g \nabla \mathbf{B}.\mathbf{x} - m g \mathbf{G}.\mathbf{x} \end{bmatrix}

where :math:`\mathbf{B}` and :math:`\mathbf{G}` are respectively the buoyancy center and the center of gravity, and
:math:`\nabla` is the volume displacement.

At each iteration, the body mesh position and orientation is then updated accordingly to the previous solutions :math:`(z, \phi, \theta)`.
The mesh is then clipped by the free-surface plane and the hydrostatic quantities (buoyancy center, restoring coefficients,
etc.) are computed. A new system can finally be solved, using the stiffness matrix newly computed and the residuals,
corresponding to the second hand term of the previous equation.

A relaxation is imposed on the solutions, by ensuring the solutions are not greater than user-defined values.

Hydrostatic quantities expressions
----------------------------------

The hydrostatic quantities are given by the following expressions

.. math::
    :nowrap:

    \begin{eqnarray}
        K_{33} &=& \rho g S_f\\
        K_{34} &=& \rho g \iint_{S_f} y dS\\
        K_{35} &=& -\rho g \iint_{S_f} x dS\\
        K_{45} &=& -\rho g \iint_{S_f} xy dS\\
        R_t &=& \frac{1}{\nabla} \iint_{S_f} y^2 dS\\
        R_l &=& \frac{1}{\nabla} \iint_{S_f} x^2 dS\\
        a &=& z_g - z_b \\
        GM_t &=& R_t - a\\
        GM_l &=& R_l - a\\
        K_{44} &=& \rho g \nabla GM_t\\
        K_{55} &=& \rho g \nabla GM_l\\
        x_f &=& -\frac{K_{35}}{K_{33}}\\
        x_f &=& \frac{K_{34}}{K_{33}}
    \end{eqnarray}

where :math:`R_t` and :math:`R_l` are respectively the transversal and longitudinal metacentric radius,
:math:`GM_t` and :math:`GM_l` are respectively the transversal and longitudinal metacentric heights,
:math:`x_f` and :math:`y_f` are the horizontal position of the center of the flotation plane.


Corrections can be added on the metacentric radii and stiffness coefficients, to express them on a particular point :math:`R = (x_R,y_R,z_R)`.


.. math::
    :nowrap:

    \begin{eqnarray}
        K_{34} &=& K_{34} - \rho g y_R  S_f\\
        K_{35} &=& K_{34} + \rho g x_R  S_f\\
        K_{45} &=& K_{45} + \rho g \left(-y_R \iint_{S_f} x dS - x_R \iint_{S_f} y dS + x_R y_R S_f \right)\\
        R_t &=& R_t + \frac{1}{\nabla} \left(- 2. y_R \iint_{S_f} y dS + y_R^2 S_f\right)\\
        R_l &=& R_l + \frac{1}{\nabla} \left(- 2. x_R \iint_{S_f} x dS + x_R^2 S_f\right)\\
    \end{eqnarray}

These corrections correspond simply to an offset in position of the body mesh.