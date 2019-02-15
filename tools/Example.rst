
Recommendations on how to write doc in Sphinx
=============================================

You can find here some notation conventions to write new entries to the theory manual. Click on the "View page source to
visualize the source code and the different sphinx commands.

You can have access to 6 levels of titles, with the following notations :

\*****
Title
\*****
Title 1
\*******
Title 2
\=======
Title 3
\-------
Title 4
\~~~~~~~
Title 5
\_______

Notation conventions
--------------------

- For vectors, use \mathbf{} and lowercase : :math:`\mathbf{u}`
- For matrix, use \mathbf{} and uppercase : :math:`\mathbf{M}`
- For tensor, use \mathcal and uppercase : :math:`\mathcal{M}`
- For inertia tensor, use \mathbb and upercase : :math:`\mathbb{I}`


Arborescence
------------

to include .rst, use

\.. toctree::
    \:maxdepth: 1

    fileName

Example of References
---------------------

In the text: \[LEONARD2001]_


at the end:
.. [LEONARD2001] Leonard, W., *Control of electrical drivers*, 3rd ed. Springer, 2001




.. _thruster:

Thruster Model
==============

The thruster model contains all elements to represent the system of propulsion of a ship. They are composed of four main items :

    - marine engine system
    - shaft speed transmission system
    - propeller model
    - force / moment model

These four elements have to be defined for each propulsion system before assignement to the ship model. These elements are defined in next sections.
    

.. image:: _images/prop_chaine.png    
    
Marine Engine
~~~~~~~~~~~~~

Two kind of marine engine can be used for the motor : 

- the electric engine 
- the diesel engine

Electric Engine
---------------

In the electric engine, the motor torque is controlled by the current, voltage and fluxes. The motor torque dynamic can be represented by a first order filter [LEONARD2001]_.

.. math:: 
    \dot{Q_m} = \frac{1}{T_m} (Q_{m_d} - Q_m)
    
where :math:`T_m` is the time response of the motor, :math:`Q_m`the motor torque and :math:`Q_{m_d}` the control command of the torque. For large propeller, the motor torque controller can be considered faster than the shaft dynamic and it can be assumed that :math:`Q_m = Q_{m_d}`.


Diesel Engine
-------------

Following [ANDERSEN1974]_ [BLANKE1981]_ , the motor dynamics can be approximated by a first order transfert function which accounts for the gradual bluid up of cylinder pressure and the discrete nature of cylinder firings.

.. math::
    Q_m(s) = e^{-s \tau_m} \frac{K_y}{1 + s T_m} Y(s)
        
where :math:`\tau_m` is the time delay, :math:`T_m` a time constant, :math:`K_y` a motor torque constant and :math:`Y(s)` the fuel index. :math:`Q_{m_d}` is given by :

.. math::
    Q_{m_d}(t) = K_y y(t)
    

In time domain, this model is equivalent to the following differential equation:

.. math::
    \dot{Q}_m = \frac{1}{T_m} (Q_{m_d} (t-\tau) - Q_m(T))
    

    
.. note::
    The time integration of the motor dynamic equation is defined in the the Update function. An Euler eplicit scheme is used to udpate the value of the motor torque :math:`Q_m`.
    

    
Shaft speed transmission
~~~~~~~~~~~~~~~~~~~~~~~~

The system of propulsion from the motor command to the angular velocity of the propeller can be represented by the next figure.


.. image:: _images/propulsion_system.png

where :math:`\omega` is the angular velocity of the propeller, :math:`Q_p` is the moment of the propeller, :math:`Q_f` is the moment due to friction, :math:`Q_m` and :math:`\omega_m` are respectively the motor torque and velocity,  :math:`Q_{m_d}` and :math:`\omega_{m_d}` are respectively the motor torque and velocity command. 

The angular velocity of the propeller :math:`\omega_m` is solution of the following differential equation :

.. math::
    J_m \dot{\omega} = Q_m - Q_f - Q_p
    
where :math:`J_m` is the total angular mass inertia of the system.


An Euler explit firt order scheme is used to solve this Ordinary Differential Equation (OED).


Propeller
~~~~~~~~~

Two kind of propeller system are represented :

    - standard propeller : fixed in the body coordinate system. The force / moment have a constant direction
    - azimuthal propeller : a rotation is allowed around the z-axis in the propulsion system's coordinate system.
    
The position and direction of the propeller :math:`(O', x', y', z')` is defined in the reference frame of the propulsion system :math:`(O, x, y, z)` as represented in next figure

.. image:: 
    _images/prop_frame.png
    :scale: 50 %
    :align: center

For the standard propeller, the thrust and moment are applied along :math:`\vec{n}` corresponding to the *x'* vector fixed in time. The azimuthal propeller can rotate along the z-axis. The direction of the thrust and moment are defined such that 

.. math::
    \vec{n} = rot_z(\alpha) \cdot \vec{x'}
   
where :math:`rot_z` is the matrix of rotation along *z* with the angle :math:`\alpha`.

.. note::
    The standard propeller model must be linked with a rudder model for ship manoeuvrability. For general concern, the rudder is defined outside the propuslion system since it can be defined without the definition of a propulsion system (for sail boat for example).



Force / Moment model
~~~~~~~~~~~~~~~~~~~~

The thrust and moment generated by the propeller along its normal direction are defined in the force / moment model. Open-water model in first quadrant and four-quadrant are used.

.. image:: _images/four_quad_model.png
    :align: center


First quadrant model
--------------------

The first-quadrant model allows to represent the behaviour of the propeller in the first quadrant only, when ship velocity is headway with thrusting ahead. 
It is based on the :math:`K_T` and :math:`K_Q` coefficients for the thrust and torque force respectively. These coefficients depend on the propeller geometry and the advance speed ratio :math:`J` defined as follows :

.. math::
    J = \frac{2 \pi V_a}{\omega D}
    
where :math:`V_a` is the axial velocity of the water through the propeller, :math:`\omega` is the angular velocity of the propeller in *rad/s* and :math:`D` the diameter of the propeller in *m*. 

The thrust :math:`T_a` and moment :math:`Q_a` of the propeller are defined as follow :

.. math:: 
    T_a = \frac{1}{4 \pi^2} \rho_w D^4 K_T(J) |\omega| \omega \\
    Q_a = \frac{1}{4 \pi^2} \rho_w D^5 K_Q(J) |\omega| \omega
    
where :math:`\rho_w`is the density of water.


:math:`K_T` and :math:`K_Q` have been computed for a large serie of propeller and Wageningen B-series can be used to estimate this parameters.


Four quadrants model
-------------------

The four-quadratn model allows to represent the behaviour of the propeller in the four quadrants. It is based on the :math:`c_T` and :math:`c_Q` coefficients which depend on the propeller blade advance angle :math:`\beta` defined such that :

.. math::
    \beta = \pi - atan2(V_a, V_p)
    
where :math:`V_a` is the axial velocity and :math:`V_p = 0.7 \pi \omega D`.

The thrust :math:`T_a` and moment :math:`Q_a` of the propeller are defined as follow :

.. math::
    T_a = \frac{1}{2} \rho_w c_T(\beta) ( V_a^2 + V_p^2) \pi R^2 \\
    Q_a = - \frac{1}{2} \rho_w c_Q(\beta) ( V_a^2 + V_p^2) \pi R^2 D
    
:math:`c_T` and :math:`c_Q` coefficients for various propellers are given by van Lammeren et al / Oosterveld, Healey et al's approximation or L-model.


Friction
~~~~~~~~

The friction model is based on a semi-empirical model presented in [PIVANO208]_.

.. math::
    Q_f(\omega) = k_{f_1} arctan( \frac{\omega}{\epsilon} ) + k_{f_2} \omega + k_{f_3} arctan( k_{f_4} \omega)
    
where :math:`k_{f_i}` and :math:`\epsilon` are constant and positive coefficients. The Coulomb effect is represented by a term in :math:`arctan` in order to avoid discontinuites in :math:`\omega = 0`. The coefficients are found by experiments.


References
~~~~~~~~~~

.. [LEONARD2001] Leonard, W., *Control of electrical drivers*, 3rd ed. Springer, 2001
.. [ANDERSEN1974] Andersen, T.E., *On dynamics of large ship diesel engines*, Master's thesis, Technical University of Denmark, 1974.
.. [BLANKE1981] Blanke, M., *Ship propulsion losses related to automatic steering and prime mover control*, PhD thesis, Technical University of Denmark, 1981.
.. [PIVANO208] Pivano, L., *Thrust estimation and control of marine propellers in four-quadrant operations*, PhD thesis, Norwegian University of Science and Technology, 2008