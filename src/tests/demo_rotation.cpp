//
// Created by frongere on 27/06/17.
//

#include "chrono/core/ChQuaternion.h"
#include "chrono/core/ChMatrix33.h"
#include "../core/FrConstants.h"
#include "../core/FrEulerAngles.h"


int main(int argc, char* argv[]) {

    auto cardan_angles = chrono::ChVector<>(10, 20, 30);
//    cardan_angles *= M_DEG;

//    // A facon chrono

    chrono::ChMatrix33<> mat1;
    double deg = M_DEG;
    mat1.Set_A_Cardano(cardan_angles * deg);
    auto quat1 = mat1.Get_A_quaternion();
    auto cardan_angles2 = mat1.Get_A_Cardano() / deg;

    // A facon frydom
    auto mat2 = frydom::euler_to_mat(cardan_angles[0], cardan_angles[1], cardan_angles[2], frydom::CARDAN, frydom::DEG);

//    auto quat2 = mat2.Get_A_quaternion();

    auto quat2 = frydom::euler_to_quat(cardan_angles[0], cardan_angles[1], cardan_angles[2], frydom::CARDAN, frydom::DEG);


    auto angles = frydom::mat_to_euler(mat2, frydom::CARDAN, frydom::DEG);

    auto angles2 = frydom::quat_to_euler(quat2, frydom::CARDAN, frydom::DEG);

    return 0;

}