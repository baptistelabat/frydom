from HydroDB.wave_drift_db import WaveDriftDB
import numpy as np
from math import *


def main():

    print("====================================================\n"
          "Test : Wave drift database\n"
          "====================================================\n")

    print("Configuration ...")
    freq = np.linspace(0.1, 2.5, 100)
    data = np.random.rand(100)

    database = WaveDriftDB()

    print("Set data ...")
    database.add_cx(freq, data, 0., unit_freq='deg')
    database.add_cx(freq, 0.1*data, 90., unit_freq='deg')
    database.add_cx(freq, -data, 0., unit_freq='deg')

    print("Set discretization ...")
    database.nb_frequencies = 100
    database.discrete_wave_dir = np.arange(0., pi, pi/5.)
    database.sym_x = True
    database.sym_y = False

    print("Initialization ...")
    database.initialize()

    print("End")
    return 0


if __name__ == "__main__":
    main()
