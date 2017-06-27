//
// Created by frongere on 27/06/17.
//

#ifndef FRYDOM_EULER_ANGLES_H
#define FRYDOM_EULER_ANGLES_H

#include <cmath>

#include "chrono/core/ChMatrix33.h"
#include "FrConstants.h"

#define M_2PI 2.*M_PI

// TODO: mettre les references utilisees (cf euler_angles.py)

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
        ZYZ,
        CARDAN,
        EULER
    };

    // =================================================================================================================
    // FUNCTIONS DECLARATIONS
    // =================================================================================================================

    template <class Real=double>
    chrono::ChMatrix33<Real> quat_to_mat(chrono::ChQuaternion<Real> quat);

    template <class Real = double>
    chrono::ChMatrix33<Real> euler_to_mat(const Real phi,
                                          const Real theta,
                                          const Real psi,
                                          EulerSeq seq = CARDAN,
                                          FrAngleUnit unit = DEG);

    template <class Real=double>
    chrono::ChMatrix33<Real> euler_to_mat(const chrono::ChVector<Real> angles,
                                          EulerSeq seq = CARDAN,
                                          FrAngleUnit unit = DEG);

    template <class Real=double>
    void eul2mat_CARDAN(chrono::ChMatrix33<Real>& rotmat,
                        Real cphi, Real sphi,
                        Real ctheta, Real stheta,
                        Real cpsi, Real spsi);


    template <class Real=double>
    chrono::ChQuaternion<Real> euler_to_quat(const Real phi,
                                             const Real theta,
                                             const Real psi,
                                             EulerSeq seq = CARDAN,
                                             FrAngleUnit unit = DEG);

    template <class Real=double>
    chrono::ChQuaternion<Real> euler_to_quat(const chrono::ChVector<Real> angles,
                                             EulerSeq seq = CARDAN,
                                             FrAngleUnit unit = DEG);

    template <class Real=double>
    void eul2quat(chrono::ChQuaternion<Real>& quat,
                  const Real cphi_2, const Real sphi_2,
                  const Real ctheta_2, const Real stheta_2,
                  const Real cpsi_2, const Real spsi_2);




    // =================================================================================================================
    // FUNCTIONS IMPLEMENTATIONS
    // =================================================================================================================

    template <class Real=double>
    chrono::ChMatrix33<Real> quat_to_mat(chrono::ChQuaternion<Real> quat) {
        auto rotmat = chrono::ChMatrix33<Real>();
        rotmat.Set_A_quaternion(quat);
        return rotmat;
    }

    template <class Real=double>
    chrono::ChMatrix33<Real> euler_to_mat(const Real phi,
                                          const Real theta,
                                          const Real psi,
                                          EulerSeq seq, FrAngleUnit unit) {

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

        phi_rad = (Real) fmod(phi_rad, M_2PI);
        theta_rad = (Real) fmod(theta_rad, M_2PI);
        psi_rad = (Real) fmod(psi_rad, M_2PI);

        Real cphi = cos(phi_rad);
        Real sphi = sin(phi_rad);
        Real ctheta = cos(theta_rad);
        Real stheta = sin(theta_rad);
        Real cpsi = cos(psi_rad);
        Real spsi = sin(psi_rad);

        auto rotmat = chrono::ChMatrix33<Real>();

        switch (seq) {
//            case XYX:
//                break;
//            case XYZ:
//                break;
//            case XZX:
//                break;
//            case XZY:
//                break;
//            case YXY:
//                break;
//            case YXZ:
//                break;
//            case YZX:
//                break;
//            case YZY:
//                break;
//            case ZXY:
//                break;
//            case ZXZ:
//                break;
//            case ZYX:
//                break;
//            case ZYZ:
//                break;
            case CARDAN:
                eul2mat_CARDAN(rotmat, cphi, sphi, ctheta, stheta, cpsi, spsi);
                break;
            case EULER:
                break;
            default:
                break;
        }  // end switch (seq)

        return rotmat;
    }

    template <class Real=double>
    chrono::ChMatrix33<Real> euler_to_mat(const chrono::ChVector<Real> angles,
                                          EulerSeq seq, FrAngleUnit unit) {
        return euler_to_mat(angles[0], angles[1], angles[2], seq, unit);
    }

    template <class Real=double>
    inline void eul2mat_CARDAN(chrono::ChMatrix33<Real>& rotmat,
                               Real cphi, Real sphi,
                               Real ctheta, Real stheta,
                               Real cpsi, Real spsi) {

        rotmat.SetElement(0, 0, ctheta*cpsi);
        rotmat.SetElement(0, 1, sphi*stheta*cpsi - cphi*spsi);
        rotmat.SetElement(0, 2, cphi*stheta*cpsi + sphi*spsi);
        rotmat.SetElement(1, 0, ctheta*spsi);
        rotmat.SetElement(1, 1, sphi*stheta*spsi + cphi*cpsi);
        rotmat.SetElement(1, 2, cphi*stheta*spsi - sphi*cpsi);
        rotmat.SetElement(2, 0, -stheta);
        rotmat.SetElement(2, 1, ctheta*sphi);
        rotmat.SetElement(2, 2, ctheta*cphi);
    }


    template <class Real=double>
    chrono::ChQuaternion<Real> euler_to_quat(const Real phi,
                                             const Real theta,
                                             const Real psi,
                                             EulerSeq seq = CARDAN,
                                             FrAngleUnit unit = DEG) {

        Real phi_2_rad, theta_2_rad, psi_2_rad;

        if (unit == DEG) {
            phi_2_rad = phi * M_2PI * 0.5;
            theta_2_rad = theta * M_2PI * 0.5;
            psi_2_rad = psi * M_2PI * 0.5;
        } else {
            phi_2_rad = phi * 0.5;
            theta_2_rad = theta * 0.5;
            psi_2_rad = psi * 0.5;
        }

        double cphi_2 = cos(phi_2_rad);
        double sphi_2 = sin(phi_2_rad);
        double ctheta_2 = cos(theta_2_rad);
        double stheta_2 = sin(theta_2_rad);
        double cpsi_2 = cos(psi_2_rad);
        double spsi_2 = sin(psi_2_rad);

        auto quat = chrono::ChQuaternion<Real>();

        switch (seq) {
//            case XYX:
//                break;
//            case XYZ:
//                break;
//            case XZX:
//                break;
//            case XZY:
//                break;
//            case YXY:
//                break;
//            case YXZ:
//                break;
//            case YZX:
//                break;
//            case YZY:
//                break;
//            case ZXY:
//                break;
//            case ZXZ:
//                break;
//            case ZYX:
//                break;
//            case ZYZ:
//                break;
            case CARDAN:
                break;
            case EULER:
                break;
            default:
                break;
        }  // end switch (seq)

        return quat;

    }

    template <class Real=double>
    chrono::ChQuaternion<Real> euler_to_quat(const chrono::ChVector<Real> angles,
                                             EulerSeq seq = CARDAN,
                                             FrAngleUnit unit = DEG) {

    }

    template <class Real=double>
    inline void eul2quat(chrono::ChQuaternion<Real>& quat,
                         const Real cphi_2, const Real sphi_2,
                         const Real ctheta_2, const Real stheta_2,
                         const Real cpsi_2, const Real spsi_2) {

        quat.e0() =  cphi_2*ctheta_2*cpsi_2 + sphi_2*stheta_2*spsi_2;
        quat.e1() = -cphi_2*stheta_2*spsi_2 + ctheta_2*cpsi_2*sphi_2;
        quat.e2() =  cphi_2*cpsi_2*stheta_2 + sphi_2*ctheta_2*spsi_2;
        quat.e3() =  cphi_2*ctheta_2*spsi_2 - sphi_2*cpsi_2*stheta_2;
    }


}  // end namespace frydom


#endif //FRYDOM_EULER_ANGLES_H
