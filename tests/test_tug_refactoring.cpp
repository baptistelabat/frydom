//
// Created by frongere on 20/09/18.
//





#include "frydom/frydom.h"

#define DBLENDL std::endl << std::endl


using namespace frydom;


int main(int argc, char* argv[]) {

    // The system
    FrOffshoreSystem_ system;


//    auto cylinder = system.NewBody();
//    makeItCylinder(cylinder, 5., 30., 100e3);
//    cylinder->SetBodyFixed(true);
//    cylinder->SetCollide(false);



    auto box = system.NewBody();
    makeItBox(box, 10, 10, 10, 1);
//    box->SetBodyFixed(true);

    box->RemoveGravity(true);


    // Playing with different methods to setup body position, velocity etc...
    box->SetAbsPosition(5, 5, 0, NWU);

    box->SetCardanAngles_DEGREES(0, 90, 90, NWU); // Tester le NED en -90, doit donner pareil...


    box->SetCOGLocalPosition(5, 5, 0, NWU); // FIXME : attention on fait quoi avec l'inertie lorsqu'on specifie une position differente du COG



    box->SetAbsRotationalVelocity(0., 0., 10*DEG2RAD, NWU);
    box->SetAbsVelocity(1, 1, 0, NWU);








    // Activating limits
    box->ActivateSpeedLimits(true);
    box->SetMaxRotationSpeed(180*DEG2RAD);  // Par defaut, la limite de vitesse en rotation est tres basse ...
    box->SetMaxSpeed(10);  // FIXME : la valeur par defaut (0.5) doit etre changee !!



//    box->SetCOGLocalPosition(1, 1, 0, NWU);




//    box->SetAbsPosition(-10, 0, 0, NWU);

//    box->SetCOGAbsPosition(0, 10, 0, NWU);
//
//
//    std::cout << box->GetAbsPosition() << std::endl << std::endl;
//    std::cout << box->GetCOGAbsPosition() << std::endl << std::endl;
//    std::cout << box->GetCOGLocalPosition() << DBLENDL;
//
//    std::cout << box->GetAbsPositionOfLocalPoint(0, -30, 0) << std::endl;



//    // Playing with frames
//    FrFrame_ frame1;
//    frame1.SetPosition(10, 5, 0, NWU);
//    FrRotation_ rot1;
//    rot1.SetCardanAngles_DEGREES(180, 0, 0, NWU);
//    frame1.SetRotation(rot1);
//
//    std::cout << frame1.GetRotation(NWU) << DBLENDL;
//
//
//    FrFrame_ frame2;
//    frame2.SetPosition(15, 0, 0, NWU);
//    FrRotation_ rot2;
//    rot2.SetCardanAngles_DEGREES(0, 0, -90, NWU);
//    frame2.SetRotation(rot2);
//
//    std::cout << frame2.GetRotation(NWU) << DBLENDL;
//
////    std::cout << frame1.GetOtherFrameRelativeTransform_WRT_ThisFrame(frame2);
//
//    std::cout << frame1.GetOtherFrameRelativeTransform_WRT_ThisFrame(frame2, NWU) << std::endl;
//
//    std::cout << frame1.GetThisFrameRelativeTransform_WRT_OtherFrame(frame2, NWU) << std::endl;












    system.Initialize();

    system.RunInViewer(100, 50, false);


//    // Set the free surface
//    system.GetEnvironment()->GetFreeSurface()->SetGrid(-200, 200, 50, -200, 300, 50);
//
//
//
//    // The current
//    system.GetEnvironment()->GetCurrent<FrUniformCurrent>()->Set(WEST, 20, KNOT, NED, GOTO);
//
//    // Building a TUG
//    auto tug = std::make_shared<FrShip>();
//    system.AddBody(tug);
//    tug->SetName("MyTug");
//    tug->SetHydroMesh("MagneViking.obj", true);
//    tug->SetMass(5e7);  // TODO: plutot dans les 5e9...
//    tug->SetInertiaXX(chrono::ChVector<>(1e8, 1e9, 1e9));
//
//    tug->SetTransverseUnderWaterArea(120.);
//    tug->SetLateralUnderWaterArea(500.);
//    tug->SetLpp(76.2);
//
//    tug->SetNEDHeading(WEST);
//    tug->SetPos(chrono::ChVector<>(0, -50, 0));
//    tug->Set3DOF_ON();
//
//    tug->SetPos_dt(chrono::ChVector<>(0*MU_KNOT, 0, 0));
//
//    // Creating a ship to tug
//    auto ship = std::make_shared<FrShip>();
//    system.AddBody(ship);
//    ship->Set3DOF_ON();
//    ship->SetName("ship");
//    ship->SetHydroMesh("MagneViking.obj", true);
//    ship->SetMass(5e7);  // TODO: plutot dans les 5e9...
//    ship->SetInertiaXX(chrono::ChVector<>(1e8, 1e9, 1e9));
//
//    ship->SetTransverseUnderWaterArea(120.);
//    ship->SetLateralUnderWaterArea(500.);
//    ship->SetLpp(76.2);
//    ship->SetPos(chrono::ChVector<>(-70, -100, 0));
//
//
//
//    // Adding a curent resistance
//    std::string filename("PolarCurrentCoeffs.yml");
//    auto current_force_tug = std::make_shared<FrCurrentForce>(filename);
//    tug->AddForce(current_force_tug);
//
//    // Adding a linear damping
//    auto lin_damping_force_tug = std::make_shared<FrLinearDamping>();
//    lin_damping_force_tug->SetDiagonalDamping(1e7, 1e7, 0, 0, 0, 1e8);
//    tug->AddForce(lin_damping_force_tug);
//
//    auto current_force_ship= std::make_shared<FrCurrentForce>(filename);
//    ship->AddForce(current_force_ship);
//
//    // Adding a linear damping
//    auto lin_damping_force_ship = std::make_shared<FrLinearDamping>();
//    lin_damping_force_ship->SetDiagonalDamping(1e7, 1e7, 0, 0, 0, 1e8);
//    ship->AddForce(lin_damping_force_ship);
//
//    // Adding a propulsion force to the ship
//    auto prop_force = std::make_shared<FrTryalForce>();
//    tug->AddForce(prop_force);
//
//
//    // Creating a fairlead on the tug
//    auto fairlead_tug = tug->CreateNode(chrono::ChVector<>(-40, 0.0, 0));
//    fairlead_tug->SetName("MyFairlead");
//
//    auto fairlead_ship = ship->CreateNode(chrono::ChVector<>(40, 0, 0));
//    fairlead_ship->SetName("Fairlead");
//
//    // Creating an other node as an anchor
////    auto anchor = std::make_shared<FrNode>();
////    anchor->SetBody(system.GetWorldBodyPtr());  // Faire une classe anchor qui sait le faire toute seule
//
////    auto anchor_pos = chrono::ChCoordsys<double>();
////    anchor_pos.pos.x() = 0;
//
//    // TODO: imposer un mouvement de l'ancre avec une fonction pour emuler un remorquage a vitesse constante
////    anchor->Impose_Abs_Coord(anchor_pos);
//
//
//
//    // Creating a catenary line
//    double Lu = 120;
//    bool elastic = true;
//    auto u = chrono::ChVector<double>(0, 0, -1);
////    double q = 616.538;
////    double q = 2000;
//    double q = 1000;
////    double q = 100;
////    double EA = 1e10;
////    double EA = 1.5708e9;
//    double EA = 1e10;
//    double A = 0.05;
//    double E = EA/A;
////    auto line = FrCatenaryLine(fairlead_tug, anchor, elastic, EA, Lu, q, u);
//    auto line = std::make_shared<FrCatenaryLine>(fairlead_tug, fairlead_ship, elastic, E, A, Lu, q, u);
//    line->Initialize();
//    system.AddLink(line);








//    system.Initialize();
//    auto app = FrIrrApp(system, 300);
//    app.Run();













//    bool viz = true;
////float friction = 0.6f;
//    double step_size = 1e-1;
//    bool capture_video = false;
//
//    // TODO: mettre la boucle suivante dans une fonction plutot que de recopier a chaque fois...
//    // Il faut:
//    // system
//    // step_size
//    // TryRealTime
//    // Des trucs a sortir a chaque pas de temps
//    if (viz) {
//
//        // Using own class for irrlicht viz
//        frydom::FrIrrApp app(&system, L"Frydom vizualization based on Irrlicht");
//        app.AddTypicalLights();
//        app.AddTypicalCamera(irr::core::vector3df(0, 0, 300), irr::core::vector3df(1, 0, -1));
//
//        // Adding the FRyDoM logo
//        auto device = app.GetDevice();
//        app.AddTypicalLogo("frydom_logo.png");
//
//        app.AssetBindAll();
//        app.AssetUpdateAll();
//
////        app.SetStepManage(true);
//        app.SetTimestep(step_size);
//        app.SetTryRealtime(true);
//
//        app.SetVideoframeSave(capture_video);
//
//        auto tug_force = tug->Get_Xforce();
//
//        while (app.GetDevice()->run()) {
//            app.BeginScene();
//            app.DrawAll();
//            app.DoStep();
//            app.EndScene();
//
//            tug_force = tug->Get_Xforce();
//            std::cout << tug_force[0] << "\t" << tug_force[1] << "\t" << tug_force[2] << std::endl;
//        }
//    }

    return 0;

}