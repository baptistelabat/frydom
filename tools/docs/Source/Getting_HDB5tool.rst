Installation Instructions
=========================


**HDB5tool** is packaged for every major OS (Unix, Windows) for both 32/64 bit systems.

.. note::

    **HDB5tool** has been tested with both Python 2.7 and 3.7 distributions.

Installing with pip
-------------------

In *tools/frydom*, do::

    pip install -e .

Please, do not forget the dot.

The following Python packages have to be installed::

    pip install numpy
    pip install scipy
    pip install vtk
    pip install rstcloth
    pip install matplotlib==2.0.0

Furthermore, the documentation generator *Sphinx* needs to be installed::

    sudo apt-get install python3-sphinx