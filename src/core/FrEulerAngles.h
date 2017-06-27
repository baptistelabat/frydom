//
// Created by frongere on 27/06/17.
//

#ifndef FRYDOM_EULER_ANGLES_H
#define FRYDOM_EULER_ANGLES_H

#include <cmath>

#include "chrono/core/ChMatrix33.h"
#include "FrConstants.h"


namespace frydom {

    enum EulerSeq {
        XYX,
        XYZ,
        XZX,
        XZY,
        YXY,
        YXZ,
        YZX,
        YZY,
        ZXY,
        ZXZ,
        ZYX,
        ZYZ
    };


    template <class Real = double>
    chrono::ChMatrix33<Real> euler_to_mat(const Real phi,
                                          const Real theta,
                                          const Real psi,
                                          EulerSeq seq = XYZ,
                                          FrAngleUnit unit = DEG) {

        auto rotmat = chrono::ChMatrix33<Real>();

        double phi_rad, theta_rad, psi_rad;
        if (unit == DEG) {
            phi_rad = phi * M_DEG;
            theta_rad = theta * M_DEG;
            psi_rad = psi * M_DEG;
        } else {
            phi_rad = phi;
            theta_rad = theta;
            psi_rad = psi;
        }

        phi_rad = fmod(phi_rad, M_2_PI);
        theta_rad = fmod(theta_rad, M_2_PI);
        psi_rad = fmod(psi_rad, M_2_PI);

        double cphi = cos(phi_rad);
        double sphi = sin(phi_rad);
        double ctheta = cos(theta_rad);
        double stheta = sin(theta_rad);
        double cpsi = cos(psi_rad);
        double spsi = sin(psi_rad);

        switch (seq) {
            case XYX:
            case XYZ:
            case XZX:
            case XZY:
            case YXY:
            case YXZ:
            case YZX:
            case YZY:
            case ZXY:
            case ZXZ:
            case ZYX:
            case ZYZ:
            default:
                return rotmat;
        }  // end switch (seq)
    }





}  // end namespace frydom




#endif //FRYDOM_EULER_ANGLES_H
