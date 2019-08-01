from setuptools import setup, find_packages

setup(

    name='cedar',

    version='1.0.0',

    packages=find_packages(),

    author='cchauvigne',

    author_email='',

    description='A python module and command line to manage data archive for FRyDoM on Amazon S3',

    #long_description=open('README.rst').read(),

    include_package_data=True,

    install_requires=['numpy', 'boto3', 'argcomplete'],

    entry_points={
        'console_scripts': ['cedar=DataPackager.cedar:main']
    },

    classifiers=[
        'Programming Language :: Python',
        'Programming Language :: Python :: 2.7',
        'Development Status :: Production',
        'Natural Language :: English',
        'Operating System :: OS Independent',
        'Topic :: Scientific/Engineering',
    ],

)
