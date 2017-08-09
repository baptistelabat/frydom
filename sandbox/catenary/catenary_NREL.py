#!/usr/bin/env python
#  -*- coding: utf-8 -*-

import numpy as np
import matplotlib.pyplot as plt

def compute_catenary_position(s, H, V, EA, L, W):
    
    ns = len(s)
    
    H_W = H/W
    
    Va = V - W*L
    Va_H = Va/H
    
    Vaws_H = Va_H + W*s/H
    
    x = H_W * ( np.arcsinh(Vaws_H) - np.arcsinh(Va_H)*np.ones((ns,)) ) + H*s/EA
    z = H_W * ( np.sqrt(1+Vaws_H*Vaws_H) - np.sqrt(1+Va_H**2*np.ones((ns,))) ) + ( Va*s+W*s*s*0.5 )/EA

    return x, z

def solve_catenary(H, V, EA, L, W, l, h, CB):
    
    X0 = [l, V]
    
    maxiter = 100
    
    def objfun(X):
        l, V = X
        


if __name__ == '__main__':
    
    CB = 1.
    EA = 4.45e6
    L = 500   # Longueur de cable
    W = 1.46  # poids lineique dans l'eau
    h = 150   # hauteur entre les deux point d'accroche
    H = 100   # tension
    
    l = 5
    V = 800
    
    l0 = l
    h0 = h
    
    s = np.arange(0, L+0.5, 1)
    
    x, z = compute_catenary_position(s, H, V, EA, L, W)
    
    plt.plot(x, z, linewidth=2, color='r')
    plt.grid()
    plt.show()
    
    l, V = solve_catenary(H, V, EA, L, W, l, h, CB)
    
