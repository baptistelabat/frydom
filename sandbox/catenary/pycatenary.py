#!/usr/bin/env python
#  -*- coding: utf-8 -*-

import numpy as np
from math import sqrt
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D

class CatenaryCable(object):
    
    
    def __init__(self, elastic=True, EA=0.,
                 L=1.,
                 p0=(0., 0., 0.), pL=(0., 0., 0.),
                 force_field=(0., 0., 0.),
                 t0 = (1., 1., 1.)
                 ):
        
        # Elasticity
        self.elastic = elastic
        self.EA = EA
        
        self.p0 = np.asarray(p0, dtype=np.float)
        
        
        # Positions
        self.p0 = np.asarray(p0, dtype=np.float)
        self.pL = np.asarray(pL, dtype=np.float)
        
        self.L = float(L)
        
        self.t0 = np.asarray(t0, dtype=np.float)
        
        self._qvec = force_field
        self.q = np.linalg.norm(force_field)
        self.u = force_field / self.q
        
        self._Umat = np.eye(3, dtype=np.float) - np.outer(u, u)
        self._sqrt_eps = sqrt(np.finfo(float).eps)
        
        # Newton-Raphson solver parameters
        self._Lmin = 1e-10
        self._tol = 1e-6
        self._itermax = 100
        self._relax = 0.1
        
        
        # TODO: calculer le residu a l'initialisation et chaque fois qu'on fait un update... --> methode update
        
    
    def _rho(self, s):
        t0_qS = self.t0 - self._qvec*s
        return np.linalg.norm(t0_qS) - np.inner(u, t0_qS)
    
    def get_unstrained_chord(self, s):
        rho_0 = self._rho(0)
        rho_s = self._rho(s)
        pc = (np.dot(self._Umat, self.t0) / self.q) * np.log(rho_s/rho_0) - \
             (self.u/self.q) * (np.linalg.norm(self.t0-self._qvec*s) - np.linalg.norm(self.t0))
        return pc
    
    def get_elastic_increment(self, s):
        return self.q*s*(self.t0/self.q - self.u*s*0.5) / self.EA
    
    def get_position(self, s):
        pos = np.zeros(3, dtype=np.float)
        pos += self.p0
        pos += self.get_unstrained_chord(s) # TODO: voir si le += marche avec le broadcast
        if self.elastic:
            pos += self.get_elastic_increment(s)
        
        return pos
    
    # def is_converged(self):
    #     err = np.linalg.norm(self.get_residual())
    #     return err < self._tol
    
    def get_residual(self):
        return self.get_position(self.L) - self.pL
    
    def eval_jacobian(self, t0=None): # FIXME: le pb est dans le calcul de cette matrice jacobienne
        t0_bak = self.t0.copy()
        
        if t0 is not None:
            self.t0 = np.asarray(t0, dtype=np.float)
        
        jac = np.zeros((3, 3), dtype=np.float)
        
        res = self.get_residual()

        # [ -1.79981338e+02   1.86621682e-02   2.20008311e+02]

        for i in xrange(3):
            t0_temp = self.t0.copy()
            h = self._sqrt_eps * self.t0[i]
            t0_temp[i] += h
            self.t0 = t0_temp
            res_temp = self.get_residual()
            jac[i] = (res_temp - res) / h
        
        self.t0 = t0_bak
        
        return jac, res

    def solve(self, p0=None, pL=None):
        
        if p0 is not None:
            self.p0 = np.asarray(p0, dtype=np.float)
            
        if pL is not None:
            self.pL = np.asarray(pL, dtype=np.float)
        
        # Computing the jacobian and the residual
        jac, res = self.eval_jacobian()
        
        # Solving the system
        delta_t0 = np.linalg.solve(jac, -res)
        
        # Updating the tension t0
        self.t0 += self._relax * delta_t0
        
        res = self.get_residual()
        err = sqrt((res*res).sum() / 3.)
        
        self._iter = 0
        
        while err > self._tol and self._iter < self._itermax:
            self._iter += 1
            
            jac, res = self.eval_jacobian()
            
            delta_t0_temp = np.linalg.solve(jac, -res)
            
            # if not np.fabs(self._relax * delta_t0_temp).max() < np.fabs(delta_t0).max():
            while np.fabs(delta_t0).max() < np.fabs(self._relax * delta_t0_temp).max():
                self._relax *= 0.5
                if self._relax < self._Lmin:
                    print "Damping too strong. No convergence\n"
                    break
                    
            delta_t0 = delta_t0_temp
            self.t0 += self._relax * delta_t0
            
            self._relax = min([1., self._relax*2.])

            res = self.get_residual()  # TODO: coupler avec le eval_jacobian
            err = sqrt((res*res).sum() / 3.)
            
        if self._iter < self._itermax:
            print "Convergence in %u iterations" % self.iter
        else:
            print "No convergence after %u iterations (maximum allowed)" % self._itermax
        
        
    def plot(self):
        
        s = np.arange(0., self.L, 0.2)
        ns = len(s)

        positions = np.zeros((ns, 3))

        pos_prec = np.zeros(3, dtype=np.float)
        cable_length = 0.

        for i, ss in enumerate(s):
            pos = self.get_position(ss)
            positions[i] = pos
    
            cable_length += np.linalg.norm(pos - pos_prec)
            pos_prec = pos.copy()

        x = positions[:, 0]
        y = positions[:, 1]
        z = positions[:, 2]

        fig = plt.figure()
        ax = fig.gca(projection='3d')
        ax.plot(x, y, z)
        plt.xlabel('x')
        plt.ylabel('y')
        plt.axis('equal')
        # plt.zlabel('z')
        plt.show()





def compute_catenary_position(s, p0, t0, EA, L, q, u):
    
    pos = np.zeros(3, dtype=np.float)
    
    # p0
    pos += p0
    
    # pc
    U = np.eye(3, dtype=np.float) - np.outer(u, u)
    
    t0n = np.linalg.norm(t0)
    
    rho_0 = t0n - np.inner(u, t0)
    rho_S = np.linalg.norm(t0-q*u*s) - np.inner(u, t0-q*u*s)
    
    pos += (np.log(rho_S/rho_0)/q) * np.dot(U, t0) - (np.linalg.norm(t0/q - u*s) - np.linalg.norm(t0/q)) * u
    
    # pe
    pos += (q*s/EA) * (t0/q - u*s*0.5)

    return pos


def rho(t0, u, qS):
    t0_qS = t0 - qS
    return np.linalg.norm(t0_qS) - np.inner(u, t0_qS)

def p(S, p0, t0, u, q, EA, elastic=True):
    pres = np.zeros(3, dtype=np.float)
    pres += p0
    pres += pc(S, t0, u, q)
    if elastic:
        pres += peps(S, t0, u, q, EA)

    return pres

def pc(S, t0, u, q):

    U = np.eye(3) - np.outer(u, u)
    rho_0 = rho(t0, u, np.zeros(3))
    rho_S = rho(t0, u, q*u*S)


    return (np.dot(U, t0) / q) * np.log(rho_S/rho_0) - (u/q) * (np.linalg.norm(t0-u*q*S) - np.linalg.norm(t0))

def peps(S, t0, u, q, EA):
    return q*S*(t0/q - u*S/2.) / EA

def jacobian(t0, u, q, L, EA, elastic=True):
    rho_0 = rho(t0, u, np.zeros(3))
    rho_L = rho(t0, u, q*u*L)

    U = np.eye(3) - np.outer(u, u)

    tL = t0 - q*u*L
    tLu = tL / np.linalg.norm(tL)
    t0u = t0 / np.linalg.norm(t0)

    jac = U * np.log(rho_L/rho_0) / q


    jac += np.outer(np.dot(U, t0), tLu/rho_L - t0u/rho_0 + (1/rho_0 - 1/rho_L)*u)

    jac += np.outer(u, t0u - tLu)

    if elastic:
        jac += np.eye(3) * L/EA

    return jac

def newton_raphson(FUN, x, lambd=0.1, itermax=100, disp=False):
    N = len(x)
    Lmin = 1e-10
    ConvCrit = 1e-6
    JAC = np.zeros((N, N), dtype=np.float)
    
    F = FUN(x)
    
    for i in xrange(N):
        xtemp = x.copy()
        h = sqrt(np.finfo(float).eps) * x[i]
        xtemp[i] += h
        JAC[i, :] = (FUN(xtemp) - F) / h
        # [[ 0.01831818 -0.00034276 -0.00093842]
        #  [ 0.01797485  0.01797665 -0.00187302]
        # [ 0.01704025  0.01704022 -0.00118828]]
    s = np.linalg.solve(JAC, -F)
    
    x += lambd*s
    F = FUN(x)
    err = sqrt((F*F).sum()/N)
    
    n = 0
    
    # if disp:
        
    while err > ConvCrit and n < itermax:
        n += 1
        
        for i in xrange(N):
            xtemp = x.copy()
            h = sqrt(np.finfo(float).eps) * x[i]
            xtemp[i] += h
            JAC[i, :] = (FUN(xtemp) - F) / h
            
        sTemp = np.linalg.solve(JAC, -F)
        
        if not np.fabs(lambd*sTemp).max() < np.fabs(s).max():
            
            while  np.fabs(s).max() < np.fabs(lambd*sTemp).max():
                
                lambd *= 0.5
                if lambd < Lmin:
                    print "Damping too strong. No convergence\n"
                    break
        
        s = sTemp
        x += lambd*s
        F = FUN(x)
        
        lambd = min([1., lambd*2.])
        
        err = sqrt((F*F).sum()/N)

    if n < itermax:
        EXITFLAG = 1
        print "Solver converged in %u iteration" % n
    else:
        EXITFLAG = -1
        print "No convergence"
    
    return x, F, JAC, EXITFLAG
    
        
def plot_cable(p0, L, t0, u, q, EA, elastic):
    
    s = np.arange(0, L, 0.1)
    ns = len(s)
    
    positions = np.zeros((ns, 3))
    
    pos_prec = np.zeros(3, dtype=np.float)
    cable_length = 0.
    
    for i, ss in enumerate(s):
        pos = p(ss, p0, t0, u, q, EA, elastic)
        positions[i] = pos
        
        cable_length += np.linalg.norm(pos-pos_prec)
        pos_prec = pos.copy()
    
    x = positions[:, 0]
    y = positions[:, 1]
    z = positions[:, 2]
    
    fig = plt.figure()
    ax = fig.gca(projection='3d')
    ax.plot(x, y, z)
    plt.xlabel('x')
    plt.ylabel('y')
    plt.axis('equal')
    # plt.zlabel('z')
    plt.show()
    
    print cable_length
    
        
if __name__ == '__main__':
    
    print 'TEST CATENAIRE'
    
    elastic = True
    
    # Position des point d'ancrage
    p0 = np.zeros(3, dtype=np.float)
    pL = np.array([180, 0, 0], dtype=np.float)
    
    L = 220 # Longueur cable
    
    # Premier guess de tension
    t0 = np.array([1, 1, 1], dtype=np.float)
    
    #
    u = np.array([0, 0, -1], dtype=np.float)
    q = 616.538
    # q = 0.1
    
    EA = 1.5708e9





    cable = CatenaryCable(elastic=True, EA=EA, L=L, p0=p0, pL=pL, force_field=q*u, t0=t0)

    cable.plot()

    cable.solve()

    cable.plot()


    # import sys
    # sys.exit(0)
    plot_cable(p0, L, t0, u, q, EA, elastic)
    
    # # Ploting the line
    # s = np.arange(0, L+0.5, 1)
    # ns = len(s)
    #
    # pos = np.zeros((ns, 3))
    #
    # ppprec = p0
    # cable_len = 0.
    #
    # for i, xs in enumerate(s):
    #     pp = compute_catenary_position(xs, p0, t0, EA, L, q, u)
    #     cable_len += np.linalg.norm(pp-ppprec)
    #
    #     ppprec = pp.copy()
    #
    #     pos[i] = pp
    #
    # print cable_len
    #
    # x = pos[:, 0]
    # z = pos[:, 2]
    # plt.plot(x, z)
    # plt.grid()
    # plt.show()
    
    # Calcul de la longueur de segment
    
    def FUN(x):
        return p(L, p0, x, u, q, EA, elastic) - pL
    
    
    sol, F, JAC, EXITFLAG = newton_raphson(FUN, t0, itermax=500)
        
    # print JAC
    # print jacobian(sol, u, q, L, EA, elastic)

    plot_cable(p0, L, t0, u, q, EA, elastic)
    
    print np.linalg.norm(t0) - q*L
    
    
    
    import sys
    sys.exit(0)
    
