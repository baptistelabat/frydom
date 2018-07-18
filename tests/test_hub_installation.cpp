//
// Created by camille on 05/07/18.
//

#include "chrono/physics/ChLinkMotorRotationSpeed.h"

#include "frydom/frydom.h"


using namespace chrono;
using namespace frydom;

int main(int argc, char* argv[]) {

    // --------------------------------------------------
    // System
    // --------------------------------------------------

    FrOffshoreSystem system;

    // --------------------------------------------------
    // Solid model
    // --------------------------------------------------

    auto barge = std::make_shared<FrHydroBody>();
    barge->SetName("Barge");
    barge->SetHydroMesh("Barge2.obj", true);
    //barge->SetRot(Q_from_AngAxis(CH_C_PI_2,VECT_X));
    system.AddBody(barge);
    barge->SetBodyFixed(true);

    auto base_crane = std::make_shared<FrBody>();
    base_crane->SetName("Base_crane");
    base_crane->SetVisuMesh("BaseCrane.obj");
    //base_crane->SetRot(Q_from_AngAxis(CH_C_PI_2,VECT_X));
    base_crane->SetPos(chrono::ChVector<double>(0., +7.5, 3.));
    //base_crane->SetPos(chrono::ChVector<double>(1., -3., 7.5));
    //base_crane->SetPos(chrono::ChVector<double>(-7.5, 0., 3.));
    system.AddBody(base_crane);

//    auto tige_crane = std::make_shared<FrBody>();
//    tige_crane->SetName("Tige_crane");
//    tige_crane->SetVisuMesh("TigeCrane.obj");
//    //tige_crane->SetCOG(chrono::ChVector<double>(0., 0., 0.));
//    tige_crane->SetPos(chrono::ChVector<double>(0., +5.5, 4.5));
//    system.AddBody(tige_crane);

    /*auto hub_box = std::make_shared<FrBody>();
    hub_box->SetName("HubBox");
    hub_box->SetVisuMesh("HubBox.obj");
    //hub_box->SetCOG(chrono::ChVector<double>(0., 0., 0.));
    //hub_box->SetPos(chrono::ChVector<double>(6.,0.,3));
    system.AddBody(hub_box);

    //barge->SetBodyFixed(true);
    //base_crane->SetBodyFixed(true);
    tige_crane->SetBodyFixed(true);
    hub_box->SetBodyFixed(true);*/

    // ---------------------------------------------
    // Markers
    // ---------------------------------------------

    auto A1_barge = barge->CreateNode(ChVector<double>(0., 7.5, 3.));
    auto A1_crane = base_crane->CreateNode(ChVector<double>(0., 0., 0.));
    //auto A2_crane = base_crane->CreateNode(ChVector<double>(2., 0., 1.5));
    //auto A2_tige = tige_crane->CreateNode(ChVector<double>(0., 0., 0.));
    //auto A3_tige = tige_crane->CreateNode(ChVector<double>(19., 0., 0.));
    //auto A4_hub = hub_box->CreateNode(ChVector<double>(0., 0., 0.));

    /*
    auto A1_barge = barge->CreateNode(ChVector<double>(0., 0., -5.));
    auto A1_crane = base_crane->CreateNode(ChVector<double>(0., 0., -5.));
    auto A2_crane = base_crane->CreateNode(ChVector<double>(0., 0., 0.));
    auto A2_tige = tige_crane->CreateNode(ChVector<double>(0., 0., 0.));
    auto A3_tige = tige_crane->CreateNode(ChVector<double>(0., 0., 0.));
    auto A4_hub = hub_box->CreateNode(ChVector<double>(0., 0., 0.));
    */
    // -----------------------------------------------
    // Link engine
    // -----------------------------------------------
/*
    auto rot_funct = std::make_shared<ChFunction_Const>();
    rot_funct->Set_yconst(0.3);

    auto motor_crane = std::make_shared<ChLinkEngine>();
    motor_crane->Initialize(A1_barge, A1_crane);
    //motor_crane->Initialize(A1_barge, A1_crane);
    motor_crane->Set_eng_mode(ChLinkEngine::ENG_MODE_SPEED);
    motor_crane->Set_spe_funct(rot_funct);
    system.Add(motor_crane);
*/
 /*
    auto motor_tige = std::make_shared<ChLinkEngine>();
    motor_tige->Initialize(A2_crane, A2_tige);
    motor_tige->Set_eng_mode(ChLinkEngine::ENG_MODE_SPEED);
    system.Add(motor_tige);
*/

    // ----------------------------------------------
    // Motor
    // ----------------------------------------------

    auto rotmotor = std::make_shared<ChLinkMotorRotationSpeed>();
    auto rwspeed = std::make_shared<ChFunction_Const>(CH_C_PI_2);

    //rotmotor->Initialize(base_crane, barge, ChFrame<>(ChVector<double>(0., 7.5, 3.)));
    //rotmotor->Initialize(base_crane, barge, true,
    //                     ChFrame<>(ChVector<double>(0., 0., 0.)),
    //                     ChFrame<>(ChVector<double>(0., 7.5, 3.))
    //
    rotmotor->Initialize(base_crane, barge, ChFrame<>(ChVector<>(0., +7.5, 3.)));

    system.Add(rotmotor);


    rotmotor->SetSpeedFunction(rwspeed);

    // -----------------------------------------------
    // Simulation
    // -----------------------------------------------

    double dt = 0.01;

    system.SetTimestepperType(chrono::ChTimestepper::Type::EULER_IMPLICIT);
    system.SetStep(dt);
    system.Initialize();

    auto app = FrIrrApp(system);
    app.AddTypicalCamera(irr::core::vector3df(0, 0, 100), irr::core::vector3df(0, 0, -1));
    app.Run();



}