Installation Instructions
=========================


HDB5tool is packaged for every major OS (Unix, Windows) for both 32/64 bit systems. It requires the use of the matplotlib library version 2.0.0 or more. 

.. warning::

    HDB5tool has only been tested with a Python 2.7 distribution.

Installing with pip
-------------------

In frydom_GPL/tools/pyHDB, do::

    pip install -e .

Please, do not forget the dot.

It is also necessary to update your Python path with the following paths::

    path/to/frydom/tools/
    path/to/frydom/tools/frydom/pyHDB/

The version of *matplotlib* needs to be 2.0.0 or higher::

    pip install matplotlib==2.0.0

The following package has to be installed::

    pip install rstcloth