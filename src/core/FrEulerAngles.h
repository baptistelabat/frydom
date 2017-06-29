//
// Created by frongere on 27/06/17.
//

#ifndef FRYDOM_EULER_ANGLES_H
#define FRYDOM_EULER_ANGLES_H

#include <cmath>

#include "chrono/core/ChMatrix33.h"
#include "../misc/FrMathUtils.h"
#include "FrConstants.h"


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
    void quat_to_axis_angle(const chrono::ChQuaternion<Real> quat,
                            chrono::ChVector<Real>& axis, Real& angle,
                            FrAngleUnit unit = DEG);

    template <class Real=double>
    chrono::ChQuaternion<Real> axis_angle_to_quat(const chrono::ChVector<Real> axis, const Real angle,
                                                  FrAngleUnit unit = DEG);

    template <class Real=double>
    chrono::ChMatrix33<Real> axis_angle_to_mat(const chrono::ChVector<Real> axis, const Real angle,
                                               FrAngleUnit unit = DEG);

    template <class Real=double>
    void mat_to_axis_angle(const chrono::ChMatrix33<Real> mat,
                           chrono::ChVector<Real>& axis, Real& angle,
                           FrAngleUnit unit = DEG);

    template <class Real=double>
    chrono::ChVector<Real> axis_angle_to_euler(const chrono::ChVector<Real> axis, const Real angle,
                                               EulerSeq seq = CARDAN, FrAngleUnit unit = DEG);

    template <class Real=double>
    void axis_angle_to_euler(const chrono::ChVector<Real> axis, const Real angle,
                             Real& phi, Real& theta, Real& psi,
                             EulerSeq seq = CARDAN, FrAngleUnit unit = DEG);

    template <class Real=double>
    void euler_to_axis_angle(const chrono::ChVector<Real> euler_angles,
                             chrono::ChVector<Real>& axis, Real& angle,
                             EulerSeq seq = CARDAN, FrAngleUnit unit = DEG);

    template <class Real=double>
    void euler_to_axis_angle(const Real phi, const Real theta, const Real psi,
                             chrono::ChVector<Real>& axis, Real& angle,
                             EulerSeq seq = CARDAN, FrAngleUnit unit = DEG);

    template <class Real=double>
    chrono::ChMatrix33<Real> quat_to_mat(const chrono::ChQuaternion<Real> quat);

    template <class Real=double>
    chrono::ChQuaternion<Real> mat_to_quat(const chrono::ChMatrix33<Real> mat);

    template <class Real=double>
    chrono::ChVector<Real> quat_to_euler(const chrono::ChQuaternion<Real> quat,
                                         EulerSeq seq = CARDAN,
                                         FrAngleUnit unit = DEG);

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
    void eul2mat_xyz(chrono::ChMatrix33<Real> &rotmat,
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
    inline void eul2quat_xyz(chrono::ChQuaternion<Real>& quat,
                             const Real cphi_2, const Real sphi_2,
                             const Real ctheta_2, const Real stheta_2,
                             const Real cpsi_2, const Real spsi_2);

    template <class Real=double>
    chrono::ChVector<Real> mat_to_euler(const chrono::ChMatrix33<Real> rotmat,
                                        EulerSeq seq = CARDAN,
                                        FrAngleUnit = DEG);

    template <class Real=double>
    void mat_to_euler(const chrono::ChMatrix33<Real> rotmat,
                      Real& phi, Real& theta, Real& psi,
                      EulerSeq seq = CARDAN,
                      FrAngleUnit = DEG);

    template <class Real=double>
    inline void mat2eul_xyz(const chrono::ChMatrix33<Real>& rotmat, chrono::ChVector<Real>& angles);

    template <class Real=double>
    inline void mat2elements(const chrono::ChMatrix33<Real>& mat,
                             Real& r00, Real& r01, Real& r02,
                             Real& r10, Real& r11, Real& r12,
                             Real& r20, Real& r21, Real& r22);


    // =================================================================================================================
    // FUNCTIONS IMPLEMENTATIONS
    // =================================================================================================================

    template <class Real=double>
    void quat_to_axis_angle(const chrono::ChQuaternion<Real> quat,
                            chrono::ChVector<Real>& axis, Real& angle,
                            FrAngleUnit unit = DEG) {
        Real q0 = quat.e0();
        Real d = 1. - q0*q0;

        if (is_close(d, 0.)) {
            angle = 0.;
            axis = chrono::VECT_Z; // by default, we say that a null rotation is around the z axis... May change
        } else {
            angle = 2. * acos(q0);
            axis = quat.GetVector() / sqrt(d);

            if (unit == DEG) {
                angle = degrees(angle);
            }
        }
    }

    template <class Real=double>
    chrono::ChQuaternion<Real> axis_angle_to_quat(const chrono::ChVector<Real> axis, const Real angle,
                                                  FrAngleUnit unit = DEG) {

        Real half_angle = 0.5 * angle;
        if (unit == DEG) {
            half_angle = radians(half_angle);
        }

        auto vector = sin(half_angle) * axis / axis.Length();

        chrono::ChQuaternion<Real> quat;
        quat.e0() = cos(half_angle);
        quat.e1() = vector[0];
        quat.e2() = vector[1];
        quat.e3() = vector[2];

        return quat;
    }

    template <class Real=double>
    chrono::ChMatrix33<Real> axis_angle_to_mat(const chrono::ChVector<Real> axis, const Real angle,
                                               FrAngleUnit unit = DEG) {
        return quat_to_mat(axis_angle_to_quat(axis, angle, unit));
    }

    template <class Real=double>
    void mat_to_axis_angle(const chrono::ChMatrix33<Real> mat,
                           chrono::ChVector<Real>& axis, Real& angle,
                           FrAngleUnit unit) {
        quat_to_axis_angle(mat_to_quat(mat), axis, angle, unit);
    }

    template <class Real=double>
    chrono::ChVector<Real> axis_angle_to_euler(const chrono::ChVector<Real> axis, const Real angle,
                                               EulerSeq seq = CARDAN, FrAngleUnit unit = DEG) {
        return quat_to_euler(axis_angle_to_quat(axis, angle, unit), seq, unit);
    }

    template <class Real=double>
    void axis_angle_to_euler(const chrono::ChVector<Real> axis, const Real angle,
                             Real& phi, Real& theta, Real& psi,
                             EulerSeq seq = CARDAN, FrAngleUnit unit = DEG) {
        auto euler_angles = axis_angle_to_euler(axis, angle, seq, unit);
        phi = euler_angles.x();
        theta = euler_angles.y();
        psi = euler_angles.z();
    }

    template <class Real=double>
    void euler_to_axis_angle(const chrono::ChVector<Real> euler_angles,
                             chrono::ChVector<Real>& axis, Real& angle,
                             EulerSeq seq = CARDAN, FrAngleUnit unit = DEG) {
        quat_to_axis_angle(euler_to_quat(euler_angles, seq, unit), axis, angle, unit);
    }

    template <class Real=double>
    void euler_to_axis_angle(const Real phi, const Real theta, const Real psi,
                             chrono::ChVector<Real>& axis, Real& angle,
                             EulerSeq seq = CARDAN, FrAngleUnit unit = DEG) {
        auto quat = euler_to_quat(phi, theta, psi, seq, unit);
        quat_to_axis_angle(quat, axis, angle, unit);
    }

    template <class Real=double>
    chrono::ChMatrix33<Real> quat_to_mat(const chrono::ChQuaternion<Real> quat) {
        auto rotmat = chrono::ChMatrix33<Real>();
        rotmat.Set_A_quaternion(quat);
        return rotmat;
    }

    template <class Real=double>
    chrono::ChQuaternion<Real> mat_to_quat(const chrono::ChMatrix33<Real> mat) {
        return mat.Get_A_quaternion();
    }

    template <class Real=double>
    chrono::ChVector<Real> quat_to_euler(const chrono::ChQuaternion<Real> quat,
                                         EulerSeq seq,
                                         FrAngleUnit unit) {
        auto mat = quat_to_mat(quat);
        return mat_to_euler(mat, seq, unit);
    }

    template <class Real=double>
    chrono::ChMatrix33<Real> euler_to_mat(const Real phi,
                                          const Real theta,
                                          const Real psi,
                                          EulerSeq seq, FrAngleUnit unit) {

        double phi_rad, theta_rad, psi_rad;
        if (unit == DEG) {
            phi_rad = radians(phi);
            theta_rad = radians(theta);
            psi_rad = radians(psi);
        } else {
            phi_rad = phi;
            theta_rad = theta;
            psi_rad = psi;
        }

        phi_rad = (Real) modulo2pi(phi_rad);
        theta_rad = (Real) modulo2pi(theta_rad);
        psi_rad = (Real) modulo2pi(psi_rad);

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
            case XYZ:
                eul2mat_xyz(rotmat, cphi, sphi, ctheta, stheta, cpsi, spsi);
                break;
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
                eul2mat_xyz(rotmat, cphi, sphi, ctheta, stheta, cpsi, spsi);
                break;
//            case EULER:
//                break;
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
    inline void eul2mat_xyz(chrono::ChMatrix33<Real> &rotmat,
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
            phi_2_rad = radians(phi) * 0.5;
            theta_2_rad = radians(theta) * 0.5;
            psi_2_rad = radians(psi) * 0.5;
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
            case XYZ:
                eul2quat_xyz(quat, cphi_2, sphi_2, ctheta_2, stheta_2, cpsi_2, spsi_2);
                break;
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
                eul2quat_xyz(quat, cphi_2, sphi_2, ctheta_2, stheta_2, cpsi_2, spsi_2);
                break;
//            case EULER:
//                break;
            default:
                break;
        }  // end switch (seq)

        return quat;

    }

    template <class Real=double>
    chrono::ChQuaternion<Real> euler_to_quat(const chrono::ChVector<Real> angles,
                                             EulerSeq seq = CARDAN,
                                             FrAngleUnit unit = DEG) {
        return euler_to_quat(angles[0], angles[1], angles[2], seq, unit);
    }

    template <class Real=double>
    inline void eul2quat_xyz(chrono::ChQuaternion<Real>& quat,
                         const Real cphi_2, const Real sphi_2,
                         const Real ctheta_2, const Real stheta_2,
                         const Real cpsi_2, const Real spsi_2) {

        quat.e0() =  cphi_2*ctheta_2*cpsi_2 + sphi_2*stheta_2*spsi_2;
        quat.e1() = -cphi_2*stheta_2*spsi_2 + ctheta_2*cpsi_2*sphi_2;
        quat.e2() =  cphi_2*cpsi_2*stheta_2 + sphi_2*ctheta_2*spsi_2;
        quat.e3() =  cphi_2*ctheta_2*spsi_2 - sphi_2*cpsi_2*stheta_2;
    }


    template <class Real=double>
    void mat_to_euler(const chrono::ChMatrix33<Real> rotmat,
                      Real& phi, Real& theta, Real& psi,
                      EulerSeq seq,
                      FrAngleUnit unit){

        chrono::ChVector<Real> angles;

        switch (seq) {
//            case XYX:
//                break;
            case XYZ:
                mat2eul_xyz(rotmat, angles);
                break;
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
                mat2eul_xyz(rotmat, angles);
                break;
//            case EULER:
//                break;
            default:
                break;
        }  // end switch (seq)

        if (unit == DEG) {
            angles = degrees(angles);
        }

        phi = angles.x();
        theta = angles.y();
        psi = angles.z();

    }

    template <class Real=double>
    chrono::ChVector<Real> mat_to_euler(const chrono::ChMatrix33<Real> rotmat,
                                        EulerSeq seq,
                                        FrAngleUnit unit){

        Real phi, theta, psi;
        mat_to_euler(rotmat, phi, theta, psi, seq, unit);

        return chrono::ChVector<Real>(phi, theta, psi);
    }

    template <class Real=double>
    inline void mat2eul_xyz(const chrono::ChMatrix33<Real>& rotmat, chrono::ChVector<Real>& angles) {
        Real r00, r01, r02, r10, r11, r12, r20, r21, r22;
        mat2elements(rotmat, r00, r01, r02, r10, r11, r12, r20, r21, r22);

        Real phi, theta, psi;

        if (r20 < 1.) {
            if (r20 > -1.) {
                theta = asin(-r20);
                psi = atan2(r10, r00);
                phi = atan2(r21, r22);
            } else { // r20 = -1
                // Not a unique solution: phi -psi = atan2(-r12, r11)
                theta = M_PI_2;
                psi = -atan2(-r12, r11);
                phi = 0.;
            }
        } else { // r20 = +1
            // Not a unique solution: phi+psi = atan2(-r12, r11)
            theta = - M_PI_2;
            psi = atan2(-r12, r11);
            psi = 0.;
        }
        angles.x() = phi;
        angles.y() = theta;
        angles.z() = psi;
    }


    template <class Real=double>
    inline void mat2elements(const chrono::ChMatrix33<Real>& mat,
                             Real& r00, Real& r01, Real& r02,
                             Real& r10, Real& r11, Real& r12,
                             Real& r20, Real& r21, Real& r22) {
        r00 = mat.GetElement(0, 0);
        r01 = mat.GetElement(0, 1);
        r02 = mat.GetElement(0, 2);
        r10 = mat.GetElement(1, 0);
        r11 = mat.GetElement(1, 1);
        r12 = mat.GetElement(1, 2);
        r20 = mat.GetElement(2, 0);
        r21 = mat.GetElement(2, 1);
        r22 = mat.GetElement(2, 2);
    }



}  // end namespace frydom


#endif //FRYDOM_EULER_ANGLES_H
