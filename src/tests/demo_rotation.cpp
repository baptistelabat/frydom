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

    auto cardan = ChVector<>(10, 20, 30);

    // euler -> quaternion
    auto quat = euler_to_quat(cardan, CARDAN, DEG);

    // quaternion -> axis angle
    ChVector<> axis;
    double angle;
    quat_to_axis_angle(quat, axis, angle, DEG);

    // axis angle -> euler (verif)
    auto cardan1 = axis_angle_to_euler(axis, angle, CARDAN, DEG);

    // axis angle -> matrix
    auto mat = axis_angle_to_mat(axis, angle, DEG);

    // matrix -> euler (verif)
    auto cardan2 = mat_to_euler(mat, CARDAN, DEG);

    // euler -> axis angle (verif)
    ChVector<> axis1;
    double angle1;
    euler_to_axis_angle(cardan2, axis1, angle1, CARDAN, DEG);

    // matrix -> axis angle
    ChVector<> axis2;
    double angle2;
    mat_to_axis_angle(mat, axis, angle, DEG);

    // axis angle -> quaternion
    auto quat1 = axis_angle_to_quat(axis2, angle2, DEG);

    // quaternion -> euler (verif)
    auto cardan3 = quat_to_euler(quat1, CARDAN, DEG);







//    auto quat2inv = quat2.GetInverse();
//    auto card = quat_to_euler(quat2inv, CARDAN, RAD);
//
//    auto mat3 = euler_to_mat(card, CARDAN, RAD);
//    mat3.FastInvert(mat3);
//    auto q = mat_to_quat(mat3);
//    auto c = quat_to_euler(q, CARDAN, DEG);




    return 0;

}