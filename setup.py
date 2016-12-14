"""Setup script for FRyDoM Core"""

from __future__ import print_function
from setuptools import setup, find_packages
import codecs
import os

HERE = os.path.abspath(os.path.dirname(__file__))


def read(*parts):
    """Return multiple read calls to different readable objects as a single
    string."""
    # intentionally *not* adding an encoding option to open
    return codecs.open(os.path.join(HERE, *parts), 'r').read()

LONG_DESCRIPTION = read('README.rst')

setup(
    name = 'frydom',
    version = '0.1',
    url = 'https://', # TODO : A completer
    author = 'Francois Rongere -- Ecole Centrale de Nantes',
    author_email = 'Francois.Rongere@ec-nantes.fr',
    description = """A framework for simulation of marine operations""",
    long_description = LONG_DESCRIPTION,
    license = 'Waiting for decision',
    keywords = 'multibody dynamics, robotics, hydrodynamics, control',
    packages = find_packages(exclude=['contrib', 'docs', 'tests*']),
    setup_requires = ['pytest-runner'],
    tests_require = ['pytest', 'pytest-cov'],
    install_requires = [
        'argparse',
        'argcomplete',
        'numpy',
        'sphinx-argparse',
        'sphinxcontrib-programoutput',
        ],
    entry_points = {
        'console_scripts': [
            'meshmagick=meshmagick:main',
        ],
    },
    classifiers = [
        'Programming Language :: Python',
        'Programming Language :: Python :: 2',
        'Development Status :: 4 - Beta',
        'Natural Language :: English',
        'Intended Audience :: Science/Research',
        'Topic :: Scientific/Engineering',
        'Topic :: Software Development :: Libraries :: Python Modules',
        'License :: OSI Approved :: CEA CNRS Inria Logiciel Libre License, version 2.1 (CeCILL-2.1)',
        'Operating System :: OS Independent',
    ],
)
