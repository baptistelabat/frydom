//
// Created by frongere on 27/06/17.
//

#include "chrono/core/ChQuaternion.h"
#include "chrono/core/ChMatrix33.h"
#include "../core/FrConstants.h"
#include "../core/FrEulerAngles.h"

using namespace chrono;
using namespace frydom;

int main(int argc, char* argv[]) {

    // TODO: gerer la requete d'expression des angles d'euler dans NWU et NED
    // TODO: gerer une expression des angles d'euler tels que phi et theta \in ]-180, 180] et psi \in [0, 360[


    auto cardan = ChVector<>(0, -90, 0);

    // euler -> quaternion
    auto quat = euler_to_quat(cardan, CARDAN, DEG);

    // quaternion -> axis angle
    ChVector<> axis;
    double angle;
    quat_to_axis_angle(quat, axis, angle, DEG);

    // axis angle -> euler (verif)
    auto cardan1 = axis_angle_to_euler(axis, angle, CARDAN, DEG);
    assert((is_close(cardan[0], cardan1[0]) &&
            is_close(cardan[1], cardan1[1]) &&
            is_close(cardan[2], cardan1[2])));

    // axis angle -> matrix
    auto mat = axis_angle_to_mat(axis, angle, DEG);

    // matrix -> euler (verif)
    auto cardan2 = mat_to_euler(mat, CARDAN, DEG);
    assert((is_close(cardan[0], cardan2[0]) &&
            is_close(cardan[1], cardan2[1]) &&
            is_close(cardan[2], cardan2[2])));

    // euler -> axis angle (verif)
    ChVector<> axis1;
    double angle1;
    euler_to_axis_angle(cardan2, axis1, angle1, CARDAN, DEG);
    assert((is_close(axis[0], axis1[0]) &&
            is_close(axis[1], axis1[1]) &&
            is_close(axis[2], axis1[2]) &&
            is_close(angle, angle1)
           ));

    // matrix -> axis angle
    ChVector<> axis2;
    double angle2;
    mat_to_axis_angle(mat, axis2, angle2, DEG);
    assert((is_close(axis[0], axis2[0]) &&
            is_close(axis[1], axis2[1]) &&
            is_close(axis[2], axis2[2]) &&
            is_close(angle, angle2)
           ));

    // axis angle -> quaternion
    auto quat1 = axis_angle_to_quat(axis2, angle2, DEG);
    assert((is_close(quat[0], quat1[0]) &&
            is_close(quat[1], quat1[1]) &&
            is_close(quat[2], quat1[2]) &&
            is_close(quat[2], quat1[2])
           ));


    // quaternion -> euler (verif)
    auto cardan3 = quat_to_euler(quat1, CARDAN, DEG);
    assert((is_close(cardan[0], cardan3[0]) &&
            is_close(cardan[1], cardan3[1]) &&
            is_close(cardan[2], cardan3[2])));








//    auto quat2inv = quat2.GetInverse();
//    auto card = quat_to_euler(quat2inv, CARDAN, RAD);
//
//    auto mat3 = euler_to_mat(card, CARDAN, RAD);
//    mat3.FastInvert(mat3);
//    auto q = mat_to_quat(mat3);
//    auto c = quat_to_euler(q, CARDAN, DEG);




    return 0;

}