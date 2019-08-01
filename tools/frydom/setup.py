from setuptools import setup, find_packages

setup(

    name='HDB5tool',

    version='2.0.0',

    packages=find_packages(),

    author='P.-Y. Wuillaume',

    author_email='pierre-yves.wuillaume@dice-engineering.com',

    description='Python module for reading Nemoh output files and writing HDB5 file for FRyDoM',

    long_description=open('README.rst').read(),

    include_package_data=True,

    install_requires=['numpy', 'matplotlib', 'h5py', 'argcomplete'],

    entry_points={
        'console_scripts': ['HDB5tool=HDB5tool.HDB5tool:main','hdb5tool=HDB5tool.HDB5tool:main']
    },

    classifiers=[
        'Programming Language :: Python',
        'Programming Language :: Python :: 3.7',
        'Development Status :: Production',
        'Natural Language :: English',
        'Operating System :: OS Independent',
        'Topic :: Scientific/Engineering',
    ],

)
