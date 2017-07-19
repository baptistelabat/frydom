#!/usr/bin/env python
#  -*- coding: utf-8 -*-

import numpy as np
import pandas as pd
import yaml
from scipy.interpolate import interp1d
import matplotlib.pyplot as plt

file = 'current_load_coefficients.csv'

if __name__ == '__main__':
    
    data = pd.read_csv(file, header=1)
    
    
    angle = np.arange(0, 182, 5, dtype=np.float)
    
    xcx = np.asarray(data['X'])
    cx = np.asarray(data['Y'])
    interpcx = interp1d(xcx, cx, fill_value="extrapolate")
    cx = interpcx(angle)
    
    xcy = np.asarray(data['X.1'])
    cy = np.asarray(data['Y.1'])
    interpcy = interp1d(xcy, cy, fill_value="extrapolate")
    cy = interpcy(angle)
    
    xcz = np.asarray(data['X.2'])
    cz = np.asarray(data['Y.2'])
    interpcz = interp1d(xcz, cz, fill_value="extrapolate")
    cz = interpcz(angle) * 1e-2
    
    # plt.plot(angle, interpcx(angle))
    # plt.plot(angle, interpcy(angle))
    # plt.plot(angle, interpcz(angle))
    # plt.grid()
    # plt.show()

    mydata = dict()
    
    coeffs = dict()
    coeffs['AngleUnit'] = 'deg'
    coeffs['Symmetric'] = True
    coeffs['Data'] = dict()
    coeffs['Data']['angle'] = angle.tolist()
    coeffs['Data']['cx'] = cx.tolist()
    coeffs['Data']['cy'] = cy.tolist()
    coeffs['Data']['cz'] = cz.tolist()

    mydata['PolarCurrentCoeffs'] = coeffs
    
    with open('PolarCurrentCoeffs.yml', 'w') as f:
    
        f.write(yaml.safe_dump(mydata, indent=4))
    
