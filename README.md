[![pipeline status](https://frydom-ce.org/ce/frydom/badges/master/pipeline.svg)](https://frydom-ce.org/ce/frydom/commits/master) [![coverage report](https://frydom-ce.org/ce/frydom/badges/master/coverage.svg)](https://frydom-ce.org/ce/frydom/commits/master)

FRyDoM-ce: Flexible and Rigid body Dynamics modeling for Marine operations  (Community Edition)
===============================================================================================

FRyDoM-ce is an open source framework for the simulation of complex systems involved in marine 
operations for Marine Renewables.
FRyDoM is a physics-based modeling and simulation framework dedicated to complex marine platforms,
structures, systems and operations. Built on a powerful multipurpose multibody dynamics numerical 
core, its object oriented modular architecture is designed to be embedded in a large range of 
application; from embedded systems up to Virtual Reality Solutions.

Implemented in modern C++, FRyDoM is released under open source GPLv3 (see LICENCE file) in 
order to provide a high-end framework to the marine community.

FRyDoM-ce (community edition) is the property of Ecole Centrale de Nantes and is maintained by 
DICE-Engineering company. D-ICE Engineering is currently developing training sessions programs 
that will help you to master FRyDoM framework. 
Please contact <a href=mailto:frydom-training@dice-engineering.com>FRyDoM training</a> for more details.

This work was carried out within the framework of the WEAMEC, West Atlantic Marine Energy Community, 
and with funding from the Pays de la Loire Region.

Starting with FRyDoM
--------------------

First, you need to add your ssh-key to your GitLab account, in order to be able to clone the 
present repository. Go to your profile settings, and click on SSH Keys in the panel on the left.
Follow the instructions provided by GitLab.

You can then follow the installation guide, provided in FRyDoM user guide (see below).

GitLab repository
-----------------

http://frydom-ce.org/ce/frydom

All users must register at http://register.frydom.org to access the code and the forum. 
If you want to contribute, you will be requested to sign a Contributor License Agreement. 
Please contact <a href=mailto:francois.rongere@dice-engineering.com>François Rongere</a> for more details.


Documentations
--------------

+ Theory manual : http://theory.frydom.org

+ User guide : http://api.frydom.org


Features
--------

+ Environment models including waves, tides, wind and current field models
+ Multibody dynamics :
    * multibody solver
    * contact detection
    * kinematic joints
    * actuators
+ Hydrodynamics :
    * Radiation (linear potential flow)
    * Hydrostatic force (linear and nonlinear)
    * Hydrostatic equilibrium solver
    * Hydrodynamic interactions between bodies
    * Waves excitation (first order and Froude-Krylov nonlinear)
    * Mean waves drift forces
    * Morison model
    * Wind & Current drag loads
    * Manoeuvring model force
    * Damping forces (linear, quadratic)
+ Cables (quasi static & dynamic FEA) for mooring system modelling, tug operations, lifting

Features in development
-----------------------

+ Cables/Seabed interactions
+ Actuators control e.g. cranes, winches, etc.
+ Non-smooth contact
+ Distributed loads on FEA cables (Morison, hydrostatic)
+ Wrapper python
