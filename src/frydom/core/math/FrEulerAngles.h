// ==========================================================================
// FRyDoM - frydom-ce.org
// 
// Copyright (c) Ecole Centrale de Nantes (LHEEA lab.) and D-ICE Engineering.
// All rights reserved.
// 
// Use of this source code is governed by a GPLv3 license that can be found
// in the LICENSE file of FRyDoM.
// 
// ==========================================================================


#ifndef FRYDOM_EULER_ANGLES_H
#define FRYDOM_EULER_ANGLES_H

#include <cmath>

#include "chrono/core/ChMatrix33.h"
#include "MathUtils/MathUtils.h"

using namespace mathutils;
// TODO: mettre les references utilisees (cf euler_angles.py)

// TODO: EulerAngles devra etre transfere dans MathUtils !!!

namespace frydom {

    enum EULER_SEQUENCE {
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


    namespace internal {
        // =================================================================================================================
        // FUNCTIONS DECLARATIONS
        // =================================================================================================================

        template<class Real=double>
        void quat_to_axis_angle(const chrono::ChQuaternion<Real>& quat,
                                chrono::ChVector<Real> &axis, Real &angle,
                                ANGLE_UNIT unit = RAD);

        template<class Real=double>
        chrono::ChQuaternion<Real> axis_angle_to_quat(const chrono::ChVector<Real>& axis, const Real angle,
                                                      ANGLE_UNIT unit = RAD);

        template<class Real=double>
        chrono::ChMatrix33<Real> axis_angle_to_mat(const chrono::ChVector<Real>& axis, const Real angle,
                                                   ANGLE_UNIT unit = RAD);

        template<class Real=double>
        void mat_to_axis_angle(const chrono::ChMatrix33<Real>& mat,
                               chrono::ChVector<Real> &axis, Real &angle,
                               ANGLE_UNIT unit = DEG);

        template<class Real=double>
        chrono::ChVector<Real> axis_angle_to_euler(const chrono::ChVector<Real>& axis, const Real angle,
                                                   EULER_SEQUENCE seq = CARDAN, ANGLE_UNIT unit = RAD);

        template<class Real=double>
        void axis_angle_to_euler(const chrono::ChVector<Real>& axis, const Real angle,
                                 Real &phi, Real &theta, Real &psi,
                                 EULER_SEQUENCE seq = CARDAN, ANGLE_UNIT unit = RAD);

        template<class Real=double>
        void euler_to_axis_angle(const chrono::ChVector<Real>& euler_angles,
                                 chrono::ChVector<Real> &axis, Real &angle,
                                 EULER_SEQUENCE seq = CARDAN, ANGLE_UNIT unit = RAD);

        template<class Real=double>
        void euler_to_axis_angle(const Real phi, const Real theta, const Real psi,
                                 chrono::ChVector<Real> &axis, Real &angle,
                                 EULER_SEQUENCE seq = CARDAN, ANGLE_UNIT unit = RAD);

        template<class Real=double>
        chrono::ChMatrix33<Real> quat_to_mat(const chrono::ChQuaternion<Real>& quat);

        template<class Real=double>
        chrono::ChQuaternion<Real> mat_to_quat(const chrono::ChMatrix33<Real>& mat);

        template<class Real=double>
        chrono::ChVector<Real> quat_to_euler(const chrono::ChQuaternion<Real>& quat,
                                             EULER_SEQUENCE seq = CARDAN,
                                             ANGLE_UNIT unit = RAD);

        template<class Real = double>
        chrono::ChMatrix33<Real> euler_to_mat(const Real phi,
                                              const Real theta,
                                              const Real psi,
                                              EULER_SEQUENCE seq = CARDAN,
                                              ANGLE_UNIT unit = RAD);

        template<class Real=double>
        chrono::ChMatrix33<Real> euler_to_mat(const chrono::ChVector<Real>& angles,
                                              EULER_SEQUENCE seq = CARDAN,
                                              ANGLE_UNIT unit = RAD);

        template<class Real=double>
        void eul2mat_xyz(chrono::ChMatrix33<Real> &rotmat,
                         Real cphi, Real sphi,
                         Real ctheta, Real stheta,
                         Real cpsi, Real spsi);


        template<class Real=double>
        chrono::ChQuaternion<Real> euler_to_quat(const Real phi,
                                                 const Real theta,
                                                 const Real psi,
                                                 EULER_SEQUENCE seq = CARDAN,
                                                 ANGLE_UNIT unit = RAD);

        template<class Real=double>
        chrono::ChQuaternion<Real> euler_to_quat(const chrono::ChVector<Real> angles,
                                                 EULER_SEQUENCE seq = CARDAN,
                                                 ANGLE_UNIT unit = RAD);

        template<class Real=double>
        inline void eul2quat_xyz(chrono::ChQuaternion<Real> &quat,
                                 const Real cphi_2, const Real sphi_2,
                                 const Real ctheta_2, const Real stheta_2,
                                 const Real cpsi_2, const Real spsi_2);

        template<class Real=double>
        chrono::ChVector<Real> mat_to_euler(const chrono::ChMatrix33<Real> rotmat,
                                            EULER_SEQUENCE seq = CARDAN,
                                            ANGLE_UNIT = RAD);

        template<class Real=double>
        void mat_to_euler(const chrono::ChMatrix33<Real> rotmat,
                          Real &phi, Real &theta, Real &psi,
                          EULER_SEQUENCE seq = CARDAN,
                          ANGLE_UNIT = RAD);

        template<class Real=double>
        inline void mat2eul_xyz(const chrono::ChMatrix33<Real> &rotmat, chrono::ChVector<Real> &angles);

        template<class Real=double>
        inline void mat2elements(const chrono::ChMatrix33<Real> &mat,
                                 Real &r00, Real &r01, Real &r02,
                                 Real &r10, Real &r11, Real &r12,
                                 Real &r20, Real &r21, Real &r22);

        // Functions relative to NED and NWU conversions

        template<class Real=double>
        inline void swap_NED_NWU(const chrono::ChVector<Real> axis, const Real angle,
                                 chrono::ChVector<Real> &new_axis, Real &new_angle);

        template<class Real=double>
        inline chrono::ChQuaternion<Real> swap_NED_NWU(const chrono::ChQuaternion<Real> quat);

        template<class Real=double>
        inline chrono::ChVector<Real> swap_NED_NWU(const chrono::ChVector<Real> angles, EULER_SEQUENCE seq);

        template<class Real=double>
        inline chrono::ChMatrix33<Real> swap_NED_NWU(const chrono::ChMatrix33<Real> mat);


        // =================================================================================================================
        // FUNCTIONS IMPLEMENTATIONS
        // =================================================================================================================


    }  // end namespace internal

}  // end namespace frydom

#include "FrEulerAngles.inl"

#endif //FRYDOM_EULER_ANGLES_H