
"""
This command line utility purpose is to calculate the maximum frequency that you can ask to BEM software for a given
mesh refinement. A common practice is to get 10 faces by wavelength to be converged with mesh refinement. The wavelength
considered is the minimum wave length of the wave spectrum which is linked to frequency through the wave dispersion
relation.
"""


if __name__ == '__main__':

    import argparse

