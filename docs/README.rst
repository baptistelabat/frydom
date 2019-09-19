FRyDoM documentation folder
===========================

This folder is organised as follow:



API documentation
-----------------

Contains the API documentation built from the source code using the Doxygen documentation generator. This documentation
is always up to date with the master branch and published at each release on the master branch to https://api.frydom.org
This documentation also contains tutorials that are commented source code showing different aspects of FRyDoM programming
usage.


To build the API guide:

.. codeblock:: bash

    cd api
    doxygen

Note that you need to have installed doxygen on your system.

Every dependencies needed by FRyDoM can be retrieved by reading the Dockerfile into docker directory...



Theory guide
------------

Contains the theory behind FRyDoM. The aim is to provide a reference on the different models used in FRyDoM but also
to provide benchmarks using FRyDoM. No mention has to be made to the code in particular as thi is the role of the API
guide. This theory guide is written in reStructuredText following the sphinx special directives and can be built with
Sphinx in different output format (html, LaTeX/PDF...). This theory guide is up to date with the develop branch so as
updates are far more frequent than API documentation. At each commit to the develop branch, this theory guide is published
to https://theory.frydom.org.


To build the theory guide:

.. codeblock:: bash

    cd theory
    make html

Note that you need to have installed both sphinx and read_the_doc theme opn your system in order to build the theory guide.

Every dependencies needed by FRyDoM can be retrieved by reading the Dockerfile into docker directory...
