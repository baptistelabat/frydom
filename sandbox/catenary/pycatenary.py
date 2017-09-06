#!/usr/bin/env python
#  -*- coding: utf-8 -*-

import numpy as np
from math import sqrt, asinh
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D

class CatenaryCable(object):
    
    
    def __init__(self, elastic=True, EA=0.,
                 L=1.,
                 p0=(0., 0., 0.), pL=(0., 0., 0.),
                 force_field=(0., 0., 0.)
                 ):
        
        # Elasticity
        self.elastic = elastic
        self.EA = EA
        
        # Positions
        self.p0 = np.asarray(p0, dtype=np.float)
        self.pL = np.asarray(pL, dtype=np.float)
        
        self.L = float(L)

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


        # First guess for the tension
        # self.t0 = np.asarray((1, 1, -1), dtype=np.float)
        p0pL = self.pL - self.p0
        lx, ly, lz = p0pL

        chord_length = np.linalg.norm(p0pL)

        v = np.cross(np.cross(self.u, p0pL/chord_length), self.u)

        if L <= chord_length:
            lambda0 = 0.2
        elif np.linalg.norm(np.cross(u, p0pL)) < 1e-4:
            lambda0 = 1e6
        else:
            lambda0 = sqrt(3 * (L*L - lz*lz) / (lx*lx+ly*ly))


        from math import tanh
        fu = -0.5 * self.q * (lz / tanh(lambda0) - L)
        fv = 0.5 * self.q * sqrt(lx*lx + ly*ly) / lambda0

        self.t0 = fu * u + fv * v


        # TODO: calculer le residu a l'initialisation et chaque fois qu'on fait un update... --> methode update
        
    
    def _rho(self, s):
        t0_qS = self.t0 - self._qvec*s
        return np.linalg.norm(t0_qS) - np.inner(u, t0_qS)
    
    def get_unstrained_chord(self, s):

        pc = - (self.u/self.q) * (np.linalg.norm(self.t0-self._qvec*s) - np.linalg.norm(self.t0))

        rho_0 = self._rho(0)
        if rho_0 > 0:
            rho_s = self._rho(s)
            if rho_s > 0.:
                pc += (np.dot(self._Umat, self.t0) / self.q) * np.log(rho_s/rho_0)

        # pc = (np.dot(self._Umat, self.t0) / self.q) * np.log(rho_s/rho_0) - \
        #      (self.u/self.q) * (np.linalg.norm(self.t0-self._qvec*s) - np.linalg.norm(self.t0))
        return pc
    
    def get_elastic_increment(self, s):
        return self.q*s*(self.t0/self.q - self.u*s*0.5) / self.EA
    
    def get_position(self, s):
        # pos = np.zeros(3, dtype=np.float)
        pos = self.p0.copy()
        pos += self.get_unstrained_chord(s) # TODO: voir si le += marche avec le broadcast
        if self.elastic:
            pos += self.get_elastic_increment(s)
        
        return pos
    
    def get_residual(self):
        return self.get_position(self.L) - self.pL

    def numerical_jacobian(self):

        t0_bak = self.t0.copy()

        jac = np.zeros((3, 3), dtype=np.float)

        res = self.get_residual()

        # Calculating the jacobian by finite difference
        for i in xrange(3):
            t0_temp = self.t0.copy()
            # Computing the step scaled by machine accuracy
            h = self._sqrt_eps * self.t0[i]
            t0_temp[i] += h
            self.t0 = t0_temp
            res_temp = self.get_residual()
            jac[i] = (res_temp - res) / h

            self.t0 = t0_bak

        return jac


    def analytical_jacobian(self):

        t0n = np.linalg.norm(self.t0) #ok

        tL = self.t0 - self._qvec * self.L #ok
        tLn = np.linalg.norm(tL) #ok

        rho_0 = t0n + self.t0[2]
        ln_q = 0.
        rho_L = 0.
        if rho_0 > 0:
            rho_L = tLn + tL[2]
            ln_q = np.log(rho_L/rho_0) / self.q

        L_EA = 0.
        if self.elastic:
            L_EA = self.L/self.EA

        jac = np.zeros((3, 3), dtype=np.float)

        for i in xrange(3):
            Uit0 = np.dot(self._Umat[i], self.t0) / self.q #ok
            for j in range(i, 3):

                jac_ij = -(tL[j]/tLn - self.t0[j]/t0n) * self.u[i] / self.q


                if rho_0 > 0.:  # Not singular
                    jac_ij += self._Umat[i, j] * ln_q
                    diff_ln = (tL[j]/tLn - self.u[j]) / rho_L - (self.t0[j]/t0n -self.u[j]) / rho_0
                    jac_ij += Uit0 * diff_ln

                # jac_ij = self._Umat[i, j] * ln_q
                #
                # diff_ln = (tL[j]/tLn - self.u[j]) / rho_L - (self.t0[j]/t0n -self.u[j]) / rho_0
                # jac_ij += Uit0 * diff_ln
                #
                # jac_ij += -(tL[j]/tLn - self.t0[j]/t0n) * self.u[i] / self.q

                jac[i, j] = jac_ij

                if i == j:
                    jac[i, j] += L_EA
                else:
                    jac[j, i] = jac[i, j]

        return jac

    def eval_jacobian(self, t0=None): # FIXME: le pb est dans le calcul de cette matrice jacobienne

        if t0 is not None:
            self.t0 = np.asarray(t0, dtype=np.float)
        
        res = self.get_residual()

        jac = self.analytical_jacobian()

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
        err = np.fabs(res).max()
        # err = sqrt((res*res).sum() / 3.)  # TODO: changer l'erreur et prendre une norme infinie

        # plt.ion() # TODO: a retirer

        self._iter = 1
        while err > self._tol and self._iter < self._itermax:

            # Display stuffs
            # self.plot()
            # plt.pause(0.1)

            self._iter += 1

            print "Iteration %u" % self._iter
            
            jac, res = self.eval_jacobian()
            
            delta_t0_temp = np.linalg.solve(jac, -res)

            # FIXME : Il n'y a a priori pas besoin de coeff de relaxation !!! Ce dernier converge tout de suite vers 1.0 !!!

            while np.fabs(delta_t0).max() < np.fabs(self._relax * delta_t0_temp).max():
                self._relax *= 0.5
                if self._relax < self._Lmin:
                    print "Damping too strong. No convergence\n"
                    break
                    
            delta_t0 = delta_t0_temp
            self.t0 += self._relax * delta_t0

            self._relax = min([1., self._relax*2.])  # Avoiding having an upscaling of the solution

            # print "RELAX = %f "% self._relax
            res = self.get_residual()  # TODO: coupler avec le eval_jacobian

            err = np.fabs(res).max()

            print "rho_0 = %f" % (np.linalg.norm(self.t0) - np.inner(self.u, self.t0))

        # If the line is in a singular configuration (straight in the load field) we project



        if self._iter < self._itermax:
            print "Convergence in %u iterations" % self._iter
        else:
            print "No convergence after %u iterations (maximum allowed)" % self._itermax

    def get_cable_length(self, ds=0.1):
        # TODO: vectoriser
        ss = np.arange(ds, self.L, ds)

        cable_length = 0.
        pos_prev = self.get_position(0.)
        for s in ss:
            pos = self.get_position(s)
            cable_length += np.linalg.norm(pos-pos_prev)
            pos_prev = pos.copy()

        return cable_length


        
    def plot(self):
        
        s = np.arange(0., self.L, 0.2)
        ns = len(s)

        positions = np.zeros((ns, 3))

        # pos_prec = np.zeros(3, dtype=np.float)
        # cable_length = 0.

        for i, ss in enumerate(s):
            pos = self.get_position(ss)
            positions[i] = pos
    
            # cable_length += np.linalg.norm(pos - pos_prec)
            # pos_prec = pos.copy()

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

    def get_tension(self, s):
        return self.t0 - self._qvec * s


if __name__ == '__main__':

    import sys

    print 'TEST CATENAIRE'

    elastic = True

    # Position des point d'ancrage
    p0 = np.zeros(3, dtype=np.float)
    pL = np.array([0, 0, -200], dtype=np.float)

    L = 200 # Longueur cable

    # Definition du champ de force uniforme constant et distribue sur le cable
    u = np.array([0, 0, -1], dtype=np.float)
    q = 616.538
    # q = 0.1

    EA = 1.5708e9


    cable = CatenaryCable(elastic=True, EA=EA, L=L, p0=p0, pL=pL, force_field=q*u)

    cable.solve()

    print cable.t0, cable.get_tension(L)

    # cable.plot()

    sys.exit(0)

    #
    #
    # yy = []
    # yy1 = []
    # yy2 = []
    #
    # xx = np.linspace(100, 250, 50)
    # # xx = np.linspace(195, 205, 100)
    #
    # rL_r0 = []
    #
    # # cable.plot()
    #
    # norm = np.linalg.norm
    #
    # # plt.ion()
    #
    # # fig = plt.figure()
    # # ax = fig.gca(projection='3d')
    # # ax.plot(x, y, z)
    # plt.xlabel('x')
    # plt.ylabel('z')
    # # plt.axis('equal')
    #
    # s = np.arange(0., L, 0.2)
    # ns = len(s)
    # positions = np.zeros((ns, 3))
    #
    # pc = []
    # pelong = []
    # clength = []
    # tension = []
    #
    # iterations = []
    #
    # for i, xl in enumerate(xx):
    #     print "\nSOLUTION FOR xl=%f" % xl
    #     cable.pL[0] = xl
    #     cable.solve()
    #     iterations.append(cable._iter)
    #
    #     # cable.plot()
    #     # for i, ss in enumerate(s):
    #     #     pos = cable.get_position(ss)
    #     #     positions[i] = pos
    #     #
    #     # x = positions[:, 0]
    #     # y = positions[:, 1]
    #     # z = positions[:, 2]
    #     #
    #     # plt.plot(x, z)
    #     # plt.show()
    #     # plt.pause(0.1)
    #     # pc.append(cable.get_unstrained_chord(L)[0])
    #     # pelong.append(cable.get_elastic_increment(L)[0])
    #     # clength.append(cable.get_cable_length())
    #
    #     # tension.append(np.linalg.norm(cable.t0))
    #     # plt.plot()
    #
    #     continue
    #
    #     # rho_0 = cable._rho(0)
    #     # rho_s = cable._rho(L)
    #     #
    #     # ll = np.log(rho_s/rho_0)
    #     #
    #     # pc1 = (np.dot(cable._Umat, cable.t0) / q) * ll
    #     #
    #     # pc2 = -(u/q) * (norm(cable.t0 - q*u*L) - norm(cable.t0))
    #     # # pc2 porte sur la composante de position suivant u
    #     #
    #     # pc = pc1 + pc2
    #     #
    #     # yy.append(norm(pc)+norm(cable.get_elastic_increment(L)))
    #     # yy1.append(norm(pc1[0]))
    #     # yy2.append(norm(pc2[0]))
    #
    # # cable.plot()
    #
    #
    #     # yy.append(np.linalg.norm(   (np.linalg.norm(cable.t0/q-u*L) - np.linalg.norm(cable.t0/q))   * u))
    #     # yy1.append(np.linalg.norm(cable.get_unstrained_chord(L)))
    #
    #     # rL_r0.append( np.log(cable._rho(L)/cable._rho(0.)) )
    #
    #
    #     # print "Cable length = %f" % cable.get_cable_length()
    #
    #     # plt.close()
    #     # yy.append(cable.get_elastic_increment(L)[0])
    #     # yy.append(cable.t0[0])
    #
    # # print "Cable length = %f" % cable.get_cable_length()
    #
    # # print yy
    # # print t0list
    # # plt.plot(xx, yy)
    # # plt.plot(xx, yy, xx, yy2)
    # # plt.plot(xx, yy, 'r')
    # # plt.plot(xx, yy1, 'g')
    # # plt.plot(xx, yy2, 'b')
    # # plt.plot(xx, rL_r0)
    # # plt.grid()
    # # plt.show()
    # # xx = np.array(xx)
    # # plt.subplot(211)
    # # plt.plot(xx, pc)
    # # plt.plot(xx, clength)
    # # plt.grid()
    # # # plt.subplot(212)
    # # # plt.plot(xx, pelong)
    # # plt.plot(xx, tension)
    # plt.plot(xx, iterations)
    # #
    # plt.grid()
    # plt.show()
