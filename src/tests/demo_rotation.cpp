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

    auto cardan_angles = ChVector<>(10, 20, 30);
//    cardan_angles *= M_DEG;

//    // A facon chrono

    ChMatrix33<> mat1;
    mat1.Set_A_Cardano(radians(cardan_angles));
    auto quat1 = mat1.Get_A_quaternion();
    auto cardan_angles2 = degrees(mat1.Get_A_Cardano());

    // A facon frydom
    auto mat2 = euler_to_mat(cardan_angles[0], cardan_angles[1], cardan_angles[2], CARDAN, DEG);

//    auto quat2 = mat2.Get_A_quaternion();

    auto quat2 = euler_to_quat(cardan_angles[0], cardan_angles[1], cardan_angles[2], CARDAN, DEG);
    auto mat3 = quat_to_mat(quat2);
    auto quat3 = mat_to_quat(mat2);

    auto card_rec = quat_to_euler(quat2, CARDAN, DEG);

    chrono::ChVector<> axis;
    double angle;

    quat_to_axis_angle(quat3, axis, angle, DEG);



    auto quatz = axis_angle_to_quat(chrono::VECT_Z, 30, DEG);
    auto quaty = axis_angle_to_quat(chrono::VECT_Y, 20, DEG);
    auto quatx = axis_angle_to_quat(chrono::VECT_X, 10, DEG);

    auto quat_total = quatx * quaty * quatz;
    auto quat_total1 = quatz * quaty * quatx;





//    auto quat2inv = quat2.GetInverse();
//    auto card = quat_to_euler(quat2inv, CARDAN, RAD);
//
//    auto mat3 = euler_to_mat(card, CARDAN, RAD);
//    mat3.FastInvert(mat3);
//    auto q = mat_to_quat(mat3);
//    auto c = quat_to_euler(q, CARDAN, DEG);




    return 0;

}