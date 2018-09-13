//
// Created by Lucas Letournel on 13/09/18.
//

#include "frydom/frydom.h"
#include "fmt/format.h"

using namespace frydom;

std::shared_ptr<FrBody> make_body(const double NEDheading_angle, const chrono::ChVector<> linearVelocity, const chrono::ChVector<> angularVelocity) {
    auto body = std::make_shared<FrBody>();
    /// Set the heading (for the frame rotation)
    auto quaternion = euler_to_quat(0., 0., -NEDheading_angle, CARDAN, RAD);
    body->SetRot(quaternion);

    /// Set body velocity
    auto bodylinearVelocity = body->TransformDirectionLocalToParent(linearVelocity);
    body->SetPos_dt(bodylinearVelocity);
    body->SetWvel_loc(angularVelocity);
    return body;
}

std::shared_ptr<FrHydroBody> make_HydroBody(const chrono::ChVector<>& unit_vector, const chrono::ChVector<> linearVelocity, const chrono::ChVector<> angularVelocity) {
    auto body = std::make_shared<FrHydroBody>();
    /// Set the heading (for the frame rotation)
    body->SetNEDHeading(unit_vector);

    /// Set body velocity
    auto bodylinearVelocity = body->TransformDirectionLocalToParent(linearVelocity);
    body->SetPos_dt(bodylinearVelocity);
    body->SetWvel_loc(angularVelocity);
    return body;
}


void print_force(std::string title , chrono::ChVector<> mforce, chrono::ChVector<> mtorque){
    fmt::print("{} \n", title);
    fmt::print("Fx  = {}, Fy  = {}, Fz  = {} \n", mforce.x(), mforce.y(), mforce.z());
    fmt::print("Mx  = {}, My  = {}, Mz  = {} \n", mtorque.x(), mtorque.y(), mtorque.z());

}

std::shared_ptr<FrLinearDamping> test_LinearDamping() {
    auto body = make_body(0*M_PI_2, chrono::ChVector<>(0,1,0),chrono::ChVector<>(0,0,1));

    auto linearForce = std::make_shared<FrLinearDamping>();
    linearForce->SetDiagonalDamping(10,10,10,10,10,10);
    linearForce->SetNonDiagonalDamping(0,5,5);
//    linearForce->SetNonDiagonalDamping(5,0,5);

    body->AddForce(linearForce);
    //linearForce->SetRelative2Current(true);
    body->Initialize();
    body->Update();

    chrono::ChVector<> mforce, mtorque;
    linearForce->GetBodyForceTorque(mforce,mtorque);
    print_force("Linear damping force", mforce, mtorque);
    return linearForce;
}

std::shared_ptr<FrLinearDamping> test_LinearDamping_HB() {
    FrOffshoreSystem system;
    /// Set the current properties
    system.GetEnvironment()->GetCurrent()->Set(NORTH, 2, MS, NED, GOTO);

    auto body = make_HydroBody(NORTH, chrono::ChVector<>(0,1,0),chrono::ChVector<>(0,0,1));
    system.Add(body);

    auto linearForce = std::make_shared<FrLinearDamping>();
    body->AddForce(linearForce);
    linearForce->SetRelative2Current(true);
    linearForce->SetDiagonalDamping(10,10,10,10,10,10);
//    linearForce->SetNonDiagonalDamping(0,5,5);
//    linearForce->SetNonDiagonalDamping(5,0,5);
    //linearForce->SetRelative2Current(true);

    body->Initialize();
    body->Update();

    chrono::ChVector<> mforce, mtorque;
    linearForce->GetBodyForceTorque(mforce,mtorque);
    print_force("Linear damping force", mforce, mtorque);
    return linearForce;
}


std::shared_ptr<FrQuadraticDamping> test_QuadraticDamping() {
    FrOffshoreSystem system;

    auto body = make_body(0*M_PI_2, chrono::ChVector<>(1,2,0),chrono::ChVector<>(0,0,1));
    system.Add(body);
    auto quadForce = std::make_shared<FrQuadraticDamping>();
    quadForce->SetDampingCoefficients(1,1,1);
    quadForce->SetProjectedSections(2,2,2);

    body->AddForce(quadForce);

    body->Initialize();
    body->Update();

    chrono::ChVector<> mforce, mtorque;
    quadForce->GetBodyForceTorque(mforce,mtorque);
    print_force("Quadratic damping force", mforce, mtorque);


    return quadForce;

}

int main() {

//    test_LinearDamping();
//    test_LinearDamping_HB();
    test_QuadraticDamping();
}
