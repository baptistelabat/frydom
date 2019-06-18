Welcome to **HDB5tool**'s documentation!
========================================

**HDB5tool** is a command line utility as well as a Python module dedicated to:
 - the post-processing of linear potential flow based solvers such as **Nemoh**;
 - the storing of the data into a *.hdb5* file to be used as input of **FRyDoM**.

**HDB5tool** also enables to change the discretization of the wave frequencies and the wave directions. It computes the Froude-Krylov loads, the infinite added-mass matrices and the impulse response functions. These latter functions may be filtered using a cutoff scaling function. It is also possible to symmetrize the hydrodynamic database if necessary. Every set of data may be plotted. 

**HDB5tool** can read every version of a *.hdb5* file and convert it into another version.

.. toctree::
   :maxdepth: 3

.. Contents:

Getting HDB5tool
================

.. toctree::
   :maxdepth: 3

   Source/Getting_HDB5tool

Using HDB5tool as a command line utility
========================================

.. toctree::
   :maxdepth: 3

   Source/Command_line

Indices and tables
==================

* :ref:`genindex`
* :ref:`modindex`
* :ref:`search`


