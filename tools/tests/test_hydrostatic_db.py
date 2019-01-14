from HydroDB.hydrostatic_db import HydrostaticDB
import numpy as np
from math import *


def main():

    print("==========================================================\n"
          "Test : hydrostatic database\n"
          "==========================================================\n")

    database = HydrostaticDB()

    database.k33 = 1e5
    database.k44 = 1e6
    database.k55 = 1e6
    database.non_diagonal = np.array([0.1, 0.2, 0.3])

    print("Matrix66:")
    print(database.matrix)

    print("Matrix33:")
    print(database.matrix33)

    return 0


if __name__ == "__main__":
    main()
