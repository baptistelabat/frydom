//
// Created by camille on 05/07/18.
//

#include <chrono/physics/ChSystemNSC.h>
#include "chrono/physics/ChLinkMotorRotationSpeed.h"

#include "frydom/frydom.h"


using namespace chrono;
using namespace frydom;

using namespace chrono::irrlicht;
// Use the main namespaces of Irrlicht
using namespace irr;
using namespace irr::core;
using namespace irr::scene;
using namespace irr::video;

void CreateStatorRotor(
        std::shared_ptr<ChBody>& mstator,
        std::shared_ptr<ChBody>& mrotor,
        ChSystem& msystem,
        const ChVector<> mpos
) {

    mstator = std::make_shared<ChBodyEasyCylinder>(0.5, 0.1, 1000, true, true);
    mstator->SetPos(mpos);
    mstator->SetRot(Q_from_AngAxis(CH_C_PI_2,VECT_X));
    mstator->SetBodyFixed(true);
    msystem.Add(mstator);

    mrotor = std::make_shared<ChBodyEasyBox>(1, 0.1, 0.1, 1000, true, true);
    mrotor->SetPos(mpos+ChVector<>(0.5,0,-0.15));
    msystem.Add(mrotor);

    auto mcolor = std::make_shared<ChColorAsset>(0.6f, 0.6f, 0.0f);
    mrotor->AddAsset(mcolor);
}

void CreateMotor(ChSystem& msystem, const ChVector<> mpos){
    std::shared_ptr<ChBody> stator1;
    std::shared_ptr<ChBody> rotor1;
    CreateStatorRotor(stator1, rotor1, msystem, mpos);

    // Create the motor
    auto rotmotor1 = std::make_shared<ChLinkMotorRotationSpeed>();

    // Connect the rotor and the stator and add the motor to the system:
    rotmotor1->Initialize(rotor1,                // body A (slave)
                          stator1,               // body B (master)
                          ChFrame<>(mpos)  // motor frame, in abs. coords
    );
    msystem.Add(rotmotor1);

    // Create a ChFunction to be used for the ChLinkMotorRotationSpeed
    auto mwspeed = std::make_shared<ChFunction_Const>(CH_C_PI_2); // constant angular speed, in [rad/s], 1PI/s =180�/s
    // Let the motor use this motion function:
    rotmotor1->SetSpeedFunction(mwspeed);
}

int main2(){

    ChSystemSMC mphysicalSystem;

    ChVector<> positionA1(0,0,0);

    CreateMotor(mphysicalSystem, positionA1);

    //
    // THE VISUALIZATION SYSTEM
    //


    // Create the Irrlicht visualization (open the Irrlicht device,
    // bind a simple user interface, etc. etc.)
    ChIrrApp application(&mphysicalSystem, L"Motors", core::dimension2d<u32>(800, 600), false);

    // Easy shortcuts to add camera, lights, logo and sky in Irrlicht scene:
    application.AddTypicalLogo();
    application.AddTypicalSky();
    application.AddTypicalLights();
    application.AddTypicalCamera(core::vector3df(0, 0, -3));
    application.AddLightWithShadow(vector3df(1.0f, 35.0f, -5.0f), vector3df(0, 0, 0), 45, 0.2, 45, 35, 512,
                                   video::SColorf(0.6f, 0.8f, 1.0f));

    // Use this function for adding a ChIrrNodeAsset to all items
    // Otherwise use application.AssetBind(myitem); on a per-item basis.
    application.AssetBindAll();

    // Use this function for 'converting' assets into Irrlicht meshes
    application.AssetUpdateAll();

    // This is to enable shadow maps (shadow casting with soft shadows) in Irrlicht
    // for all objects (or use application.AddShadow(..) for enable shadow on a per-item basis)
    application.AddShadowAll();


    //
    // THE SOFT-REAL-TIME CYCLE
    //


    // Modify some setting of the physical system for the simulation, if you want
    mphysicalSystem.SetSolverType(ChSolver::Type::SOR);
    mphysicalSystem.SetMaxItersSolverSpeed(50);

    application.SetTimestep(0.005);
    application.SetTryRealtime(true);


    while (application.GetDevice()->run()) {
        application.BeginScene(true, true, SColor(255, 140, 161, 192));

        application.DrawAll();

        application.DoStep();

        application.EndScene();
    }
}



int main(int argc, char* argv[]) {

    // --------------------------------------------------
    // System
    // --------------------------------------------------

    FrOffshoreSystem system;
    //ChSystemSMC system;

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
    base_crane->SetPos(chrono::ChVector<double>(0., +7.5, 3.15));
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

    //auto A1_barge = barge->CreateNode(ChVector<double>(0., 7.5, 3.));
    //auto A1_crane = base_crane->CreateNode(ChVector<double>(0., 0., 0.));
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
    // Motorœ
    // ----------------------------------------------

    auto rotmotor = std::make_shared<ChLinkMotorRotationSpeed>();

    rotmotor->Initialize(base_crane, barge, ChFrame<>(ChVector<>(0., +7.5, 3.)));

    //system.Add(rotmotor);

    auto rwspeed = std::make_shared<ChFunction_Const>(CH_C_PI_2);
    rotmotor->SetSpeedFunction(rwspeed);

    // ----------------------------------------------
    // TUTO
    // ----------------------------------------------

    ChVector<> positionA1(0,0,-1);
    CreateMotor(system,positionA1);

    // -----------------------------------------------
    // Simulation
    // -----------------------------------------------

    double dt = 0.01;

    system.SetTimestepperType(chrono::ChTimestepper::Type::EULER_IMPLICIT);
    system.SetStep(dt);
    system.Initialize();

    auto app = FrIrrApp(system);
    app.AddTypicalCamera(irr::core::vector3df(1, 1, -3), irr::core::vector3df(0, 0, -1));
    app.Run();

    /*// -----------------------------------------------
    // Simulation
    // -----------------------------------------------

    double dt = 0.005;

    system.SetTimestepperType(chrono::ChTimestepper::Type::EULER_IMPLICIT);
    system.SetStep(dt);

    auto app = chrono::irrlicht::ChIrrApp(&system,L"FRyDoM viewer",irr::core::dimension2d<irr::u32>(800, 600),
                               false,false,true,irr::video::EDT_OPENGL);


    // Easy shortcuts to add camera, lights, logo and sky in Irrlicht scene:
    app.AddTypicalLogo();
    app.AddTypicalSky();
    app.AddTypicalLights();
    app.AddLightWithShadow(vector3df(1.0f, 35.0f, -5.0f), vector3df(0, 0, 0), 45, 0.2, 45, 35, 512,
                                   video::SColorf(0.6f, 0.8f, 1.0f));
    app.AddTypicalCamera(irr::core::vector3df(1, 1, -30),irr::core::vector3df(0, 0, -1));

    // Use this function for adding a ChIrrNodeAsset to all items
    // Otherwise use application.AssetBind(myitem); on a per-item basis.
    app.AssetBindAll();

    // Use this function for 'converting' assets into Irrlicht meshes
    app.AssetUpdateAll();

    // This is to enable shadow maps (shadow casting with soft shadows) in Irrlicht
    // for all objects (or use application.AddShadow(..) for enable shadow on a per-item basis)
    app.AddShadowAll();

    app.SetTimestep(0.005);
    app.SetTryRealtime(true);


    while (app.GetDevice()->run()) {
        app.BeginScene(true, true, SColor(255, 140, 161, 192));

        app.DrawAll();

        app.DoStep();

        app.EndScene();
    }*/


}