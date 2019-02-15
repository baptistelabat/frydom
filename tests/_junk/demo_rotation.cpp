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

#include "frydom/frydom.h"

using namespace chrono;
using namespace frydom;

int main(int argc, char* argv[]) {

    // TODO: gerer la requete d'expression des angles d'euler dans NWU et NED
    // TODO: gerer une expression des angles d'euler tels que phi et theta \in ]-180, 180] et psi \in [0, 360[

    auto cardan = ChVector<>(10, 20, 30);

    // TODO: mettre ce qui suit en fonction et appeler pour differentes valeurs d'angle de cardan

    // euler -> quaternion
    auto quat = euler_to_quat(cardan, CARDAN, DEG);

    // quaternion -> axis angle
    ChVector<> axis;
    double angle;
    quat_to_axis_angle(quat, axis, angle, DEG);

    // axis angle -> euler (verif)
    auto cardan1 = axis_angle_to_euler(axis, angle, CARDAN, DEG);
    assert((IsClose(cardan[0], cardan1[0]) &&
            IsClose(cardan[1], cardan1[1]) &&
            IsClose(cardan[2], cardan1[2])));

    // axis angle -> matrix
    auto mat = axis_angle_to_mat(axis, angle, DEG);

    // matrix -> euler (verif)
    auto cardan2 = mat_to_euler(mat, CARDAN, DEG);
    assert((IsClose(cardan[0], cardan2[0]) &&
            IsClose(cardan[1], cardan2[1]) &&
            IsClose(cardan[2], cardan2[2])));

    // euler -> axis angle (verif)
    ChVector<> axis1;
    double angle1;
    euler_to_axis_angle(cardan2, axis1, angle1, CARDAN, DEG);
    assert((IsClose(axis[0], axis1[0]) &&
            IsClose(axis[1], axis1[1]) &&
            IsClose(axis[2], axis1[2]) &&
            IsClose(angle, angle1)));

    // matrix -> axis angle
    ChVector<> axis2;
    double angle2;
    mat_to_axis_angle(mat, axis2, angle2, DEG);
    assert((IsClose(axis[0], axis2[0]) &&
            IsClose(axis[1], axis2[1]) &&
            IsClose(axis[2], axis2[2]) &&
            IsClose(angle, angle2)));

    // axis angle -> quaternion
    auto quat1 = axis_angle_to_quat(axis2, angle2, DEG);
    assert((IsClose(quat[0], quat1[0]) &&
            IsClose(quat[1], quat1[1]) &&
            IsClose(quat[2], quat1[2]) &&
            IsClose(quat[2], quat1[2])));


    // quaternion -> euler (verif)
    auto cardan3 = quat_to_euler(quat1, CARDAN, DEG);
    assert((IsClose(cardan[0], cardan3[0]) &&
            IsClose(cardan[1], cardan3[1]) &&
            IsClose(cardan[2], cardan3[2])));

    // Tests for conversion between NED and NWU
    // quaternion
    auto quat2 = swap_NED_NWU(quat);

    // Angle axis
    ChVector<double> axis3;
    double angle3;
    swap_NED_NWU(axis, angle, axis3, angle3);
    auto quat3 = axis_angle_to_quat(axis3, angle3, DEG);
    assert((IsClose(quat3[0], quat2[0]) &&
            IsClose(quat3[1], quat2[1]) &&
            IsClose(quat3[2], quat2[2]) &&
            IsClose(quat3[3], quat2[3])));

    // cardan
    auto cardan4 = swap_NED_NWU(cardan, CARDAN);
    auto quat4 = euler_to_quat(cardan4, CARDAN, DEG);
    assert((IsClose(quat4[0], quat2[0]) &&
            IsClose(quat4[1], quat2[1]) &&
            IsClose(quat4[2], quat2[2]) &&
            IsClose(quat4[3], quat2[3])));

    // matrix
    auto mat1 = swap_NED_NWU(mat);
    auto quat5 = mat_to_quat(mat1);
    assert((IsClose(quat5[0], quat2[0]) &&
            IsClose(quat5[1], quat2[1]) &&
            IsClose(quat5[2], quat2[2]) &&
            IsClose(quat5[3], quat2[3])));

#ifdef Debug
    std::cout << "happy" << std::endl;
#endif
    return 0;
}