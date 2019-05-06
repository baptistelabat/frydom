//
// Created by lletourn on 06/05/19.
//


#include "chrono/physics/ChSystemNSC.h"
#include "chrono/physics/ChBodyEasy.h"
#include "chrono/assets/ChTexture.h"
#include "chrono/motion_functions/ChFunction_Sine.h"
#include "chrono/physics/ChLinkMotorRotationAngle.h"
#include "chrono/physics/ChLinkMotorRotationSpeed.h"
#include "chrono/physics/ChLinkMotorRotationTorque.h"
#include "chrono/physics/ChLinkMotorRotationDriveline.h"
#include "chrono/physics/ChLinkMotorLinearPosition.h"
#include "chrono/physics/ChLinkMotorLinearSpeed.h"
#include "chrono/physics/ChLinkMotorLinearForce.h"
#include "chrono/physics/ChLinkMotorLinearDriveline.h"
#include "chrono/physics/ChShaftsMotorSpeed.h"
#include "chrono/physics/ChShaftsMotorAngle.h"
#include "chrono/physics/ChShaftsPlanetary.h"
#include "chrono/physics/ChShaftsGear.h"
#include "chrono_irrlicht/ChIrrApp.h"
// Use the namespaces of Chrono
using namespace chrono;
using namespace chrono::irrlicht;
// Use the main namespaces of Irrlicht
using namespace irr;
using namespace irr::core;
using namespace irr::scene;
using namespace irr::video;

// Shortcut function that creates two bodies (a stator and a rotor) in a given position,
// just to simplify the creation of multiple linear motors in this demo
// (skip this and go to main() for the tutorial)
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

int main(int argc, char* argv[]) {
    GetLog() << "Copyright (c) 2017 projectchrono.org\nChrono version: " << CHRONO_VERSION << "\n\n";
    // Create a ChronoENGINE physical system
    ChSystemNSC mphysicalSystem;
    // Create a floor that is fixed (that is used also to represent the absolute reference)
    auto floorBody = std::make_shared<ChBodyEasyBox>(20, 2, 20, 3000, true, true);
    floorBody->SetPos(ChVector<>(0, -2, 0));
    floorBody->SetBodyFixed(true);
    mphysicalSystem.Add(floorBody);
    auto mtexture = std::make_shared<ChTexture>();
    mtexture->SetTextureFilename(GetChronoDataFile("blu.png"));
    floorBody->AddAsset(mtexture);
    // In the following we will create different types of motors
    // - rotational motors: examples A.1, A.2, etc.
    // - linear motors, examples B.1, B.2 etc.


    // EXAMPLE A.1
    //
    // - class:   ChLinkMotorRotationSpeed
    // - type:    rotational motor
    // - control: impose a time-dependent speed=v(t)
    //
    // This is a simple type of rotational actuator. It assumes that
    // you know the exact angular speed of the rotor respect to the stator,
    // as a function of time:   angular speed = w(t).
    // Use this to simulate fans, rotating cranks, etc.
    // Note: this is a rheonomic motor that enforces the motion
    // geometrically; no compliance is allowed, this means that if the
    // rotating body hits some hard contact, the solver might give unpredictable
    // oscillatory or diverging results because of the contradiction.
    ChVector<> positionA1(0., 2., 0.);
    std::shared_ptr<ChBody> stator1;
    std::shared_ptr<ChBody> rotor1;
    CreateStatorRotor(stator1, rotor1, mphysicalSystem, positionA1);

    auto rev1 = std::make_shared<ChLinkRevolute>();
    rev1->Initialize(rotor1, stator1, ChFrame<>(positionA1));
    mphysicalSystem.Add(rev1);

    // Create the motor
    auto rotmotor1 = std::make_shared<ChLinkMotorRotationSpeed>();
    // Connect the rotor and the stator and add the motor to the system:
    rotmotor1->Initialize(rotor1,                // body A (slave)
                          stator1,               // body B (master)
                          ChFrame<>(positionA1)  // motor frame, in abs. coords
    );
    mphysicalSystem.Add(rotmotor1);
    // Create a ChFunction to be used for the ChLinkMotorRotationSpeed
    auto mwspeed = std::make_shared<ChFunction_Const>(CH_C_PI_2); // constant angular speed, in [rad/s], 1PI/s =180�/s
    // Let the motor use this motion function:
    rotmotor1->SetSpeedFunction(mwspeed);
    rotmotor1->SetSpindleConstraint(false, false, false, false, false);

// The ChLinkMotorRotationSpeed contains a hidden state that performs the time integration
// of the angular speed setpoint: such angle is then imposed to the
// constraint at the positional level too, thus avoiding angle error
// accumulation (angle drift). Optionally, such positional constraint
// level can be disabled as follows:
//
//rotmotor1->SetAvoidAngleDrift(false);

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
    application.AddTypicalCamera(core::vector3df(1, 3, -7));
    application.AddLightWithShadow(vector3df(20.0f, 35.0f, -25.0f), vector3df(0, 0, 0), 55, 20, 55, 35, 512,
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
        // Example B.6 requires the setpoint to be changed in the simulation loop:
        // for example use a clamped sinusoid, just for fun:
        double t  = mphysicalSystem.GetChTime();
        double Sp = ChMin(ChMax(2.6*sin(t*1.8),-1.4),1.4);
//        motor6setpoint->SetSetpoint(Sp, t);
        application.DoStep();
        application.EndScene();
    }
    return 0;

}