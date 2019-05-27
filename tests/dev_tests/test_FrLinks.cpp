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

#include <chrono/physics/ChBodyEasy.h>
#include "frydom/frydom.h"
//#include "gtest/gtest.h"

using namespace frydom;

int main() {

    FrOffshoreSystem system;
    system.GetEnvironment()->ShowFreeSurface(false);
//    system.GetEnvironment()->GetOcean()->GetSeabed()->SetBathymetry(-50,NWU);
    system.GetEnvironment()->ShowSeabed(false);
    system.SetName("Links");

//    system.GetWorldBody()->AddSphereShape(10.);

    // Body1 definition
    auto body1 = system.NewBody();
    body1->SetName("1");
    makeItBox(body1, 20, 10, 2, 1000);
    body1->AllowCollision(false);
    body1->SetColor(MediumVioletRed);

    body1->SetPosition(Position(5.,5.,5.),NWU);


    // Apply a random translation and rotation to the body to check if the assembly is done correctly
//    body1->TranslateInWorld(9.,-6.,25, NWU);
//    Direction randomDir(1.,5.,-9.); randomDir.normalize();
//    FrRotation randomRotation(randomDir,0.2954,NWU);
//    body1->Rotate(randomRotation);

    // Revolute link between body1 and world
    auto node1 = body1->NewNode();
    node1->TranslateInBody(10, 5, 0, NWU);
    node1->ShowAsset(true);
    node1->GetAsset()->SetSize(10);
//    node1->RotateAroundXInBody(90*DEG2RAD, NWU);
//    node1->RotateAroundZInBody(90*DEG2RAD, NWU);


    auto nodeWorld = system.GetWorldBody()->NewNode();
    nodeWorld->ShowAsset(true);
    nodeWorld->GetAsset()->SetSize(10);
//    nodeWorld->TranslateInWorld(10, 0, 0, NWU);
//    nodeWorld->RotateAroundXInBody(45*DEG2RAD, NWU);
//    nodeWorld->RotateAroundZInBody(90*DEG2RAD, NWU);

    auto point1 = std::make_shared<FrPoint>(node1);
    auto pointWorld = std::make_shared<FrPoint>(nodeWorld);

    auto axis1 = std::make_shared<FrAxis>(node1,XAXIS);
    auto axisWorld = std::make_shared<FrAxis>(nodeWorld,XAXIS);

    auto plane1 = std::make_shared<FrPlane>(node1,YAXIS);
    auto planeWorld = std::make_shared<FrPlane>(nodeWorld,ZAXIS);

    auto test = std::make_shared<FrConstraintDistanceToAxis>(point1, axisWorld, &system, 10.);
    //test->SetFlipped(true);
//    test->SetDistance(0.);
    system.AddLink(test);

//    auto body2 = system.NewBody();
//    body2->SetName("1");
//    makeItBox(body2, 20, 10, 2, 1000);
//    body2->AllowCollision(false);
//    body2->SetColor(Green);
//    body2->SetPosition(Position(10,10,0), NWU);
//
//    auto node2 = body2->NewNode();
////    node2->TranslateInBody(-10,-10,0,NWU);
//
//    auto constraintPointOnPlane = make_pointOnPlane_constraint(nodeWorld, node2, &system);

//    auto constraint = std::make_shared<chrono::ChLinkRevoluteTranslational>();
//////    constraint->Initialize(box, floor, true, ChVector<>(0,-.5,0), ChVector<>(0,.5,0), ChVector<>(0,-1,0), ChVector<>(0,1,0));
//    constraint->Initialize(system.GetWorldBody()->GetChronoBody(), body1->GetChronoBody(), true,
//            chrono::ChVector<>(0,0,0), chrono::ChVector<>(0,1,0),
//                    chrono::ChVector<>(-10,-5,0), chrono::ChVector<>(1,0,0), chrono::ChVector<>(0,0,1), false, 0.);
////    constraint->SetFlipped(true);
//    system.GetChronoSystem()->AddLink(constraint);



//    auto rev1 = make_revolute_link(node1, nodeWorld, &system);
//
//    auto motor1 = rev1->Motorize(POSITION);
//
//    auto t = new_var("t");
////    motor1->SetMotorFunction(0.1*MU_PI * sin(t));
//    motor1->SetMotorFunction(FrConstantFunction(0.));


    // Body 2 definition (linked body)
//    auto body2 = system.NewBody();
//    body2->SetName("2");
//    makeItBox(body2, 2, 2, 40, 2000);
//    body2->SetColor(Black);
////    body2->TranslateInWorld(10, 5, 0, NWU);
//
////    // Apply a random translation and rotation to the body to check if the assembly is done correctly
////    body2->TranslateInWorld(-2.,7.,8, NWU);
////    randomDir = Direction(6.,-8.,45); randomDir.normalize();
////    randomRotation = FrRotation(randomDir,0.9687,NWU);
////    body2->Rotate(randomRotation);
////
//    // Prismatic link between body1 and body2
//    auto m1 = body1->NewNode();
//    m1->TranslateInBody(10, 5, -1, NWU);
//
//    auto m2 = body2->NewNode();
//    m2->TranslateInBody(0, 0, 0, NWU);
//
//    auto prismaticLink = make_prismatic_link(m1, m2, &system);
//    prismaticLink->SetSpringDamper(2e3, 1e4);
//    prismaticLink->SetRestLength(0.);
//
//    auto motor2 = prismaticLink->Motorize(POSITION);
////    motor2->SetMotorFunction(10*sin(t));
//    motor2->SetMotorFunction(FrConstantFunction(-10.));

//    auto fixedLink = make_fixed_link(m1, m2, &system);

//    auto cylindricalLink = make_cylindrical_link(m1, m2, &system);

//    auto sphericalLink = make_spherical_link(m1, m2, &system);

//    system.RemoveLink(prismaticLink);

//    auto test = system.GetWorldBody()->NewNode();
//    test->RotateAroundYInBody(45*DEG2RAD, NWU);

//    auto perpendicularConstraint = make_perpendicular_constraint(test, m2, &system);
//    auto parallelConstraint = make_parallel_constraint(test, m2, &system);
//    auto planeOnPlaneConstraint = make_planeOnPlane_constraint(test,m2,&system);
//    auto pointOnPlaneConstraint = make_pointOnPlane_constraint(m2,test,&system);
//    auto pointOnLineConstraint = make_pointOnLine_constraint(m2, test, &system);

//    // Body 3 definition
//    auto body3 = system.NewBody();
//    body3->SetName("3");
//    makeItBox(body3, 2, 2, 6, 500);
//    body3->AllowCollision(false);
//    body3->SetColor(Red);
////    body3->TranslateInWorld(-10, 5, -1, NWU);
//
//    // Apply a random translation and rotation to the body to check if the assembly is done correctly
//    body3->TranslateInWorld(0.,1.,-9, NWU);
//    randomDir = Direction(26.,98.,5); randomDir.normalize();
//    randomRotation = FrRotation(randomDir,-0.57,NWU);
//    body3->Rotate(randomRotation);
//
//    // Revolute link between body1 and body3
//    auto m3 = body1->NewNode();
//    m3->TranslateInBody(-10, 5, -1, NWU);
//
//    auto m4 = body3->NewNode();
//
//    auto revoluteLink = make_revolute_link(m3, m4, &system);
//    revoluteLink->SetSpringDamper(1e4, 1e1);
//    revoluteLink->SetRestAngle(180*DEG2RAD);

    system.Initialize();
    system.GetChronoSystem()->DoFullAssembly();

    // Run the simulation (or visualize the assembly)
    system.SetTimeStep(0.01);
    system.RunInViewer(0, 50, true);
//    system.Visualize(50, false);

    return 0;
}

//// Use the namespaces of Chrono
//using namespace chrono;
//using namespace chrono::irrlicht;
//// Use the main namespaces of Irrlicht
//using namespace irr;
//using namespace irr::core;
//using namespace irr::scene;
//using namespace irr::video;
//using namespace irr::io;
//using namespace irr::gui;
//
//int main() {
//
//
//    ChSystemSMC my_system;
//    double TimeStep = 1e-3;
//    my_system.Set_G_acc(ChVector<>(0, 9.8066, 0));
////    my_system.SetSolverType(ChSolver::Type::PMINRES);
////    my_system.SetTimestepperType(ChTimestepper::Type::EULER_IMPLICIT_LINEARIZED);
////    my_system.SetMaxiter(400);              // default 6
////    my_system.SetMaxItersSolverSpeed(500);  // default 30
////    my_system.SetMaxItersSolverStab(250);   // default 10
////    my_system.SetTol(1e-3);
//    my_system.SetStep(TimeStep);
//
//    auto Fixed= std::make_shared<ChBodyEasyBox>(0.1, 0.1, 0.1, 1, false, true);
//
//    my_system.Add(Fixed);
//    Fixed->SetPos(ChVector<>(0, 1, 0));
//    Fixed->SetBodyFixed(true);
//
//    auto body = std::make_shared<ChBodyEasyBox>(0.05, 1, 0.05, 1, false, true);
//    body->SetMass(23.5);
//    body->SetPos(ChVector<>(0, 0.5, 0));
//    my_system.Add(body);
//    body->SetId(99);
//
//    auto link = std::make_shared<ChLinkLockRevolutePrismatic>();
//    link->GetLimit_X()->Set_active(true);
//    link->GetLimit_X()->Set_min(-0.1);
//    link->GetLimit_X()->Set_max(0.1);
//    link->Initialize(Fixed, body, Coordsys(ChVector<>(0, 1, 0),Q_from_AngZ(CH_C_PI_2)));
//    my_system.AddLink(link);
//
////    RevolutePrismaticlinks.push_back(link);
////    rotateSystem(ChVector<>(0, 1, 0), 0.174533, my_system);
//
//    ChIrrApp application(&my_system, L"PlaneOnPlane test", core::dimension2d<u32>(800, 600), false, true);
//
//    // Easy shortcuts to add camera, lights, logo and sky in Irrlicht scene:
//    ChIrrWizard::add_typical_Logo(application.GetDevice());
//    ChIrrWizard::add_typical_Sky(application.GetDevice());
//    ChIrrWizard::add_typical_Lights(application.GetDevice());
//    ChIrrWizard::add_typical_Camera(application.GetDevice(), core::vector3df(0, 0, -6));
//    // Bind assets
//    application.AssetBindAll();
//    application.AssetUpdateAll();
//
//    while (application.GetDevice()->run()) {
//        // Irrlicht must prepare frame to draw
//        application.BeginScene(true, true, SColor(255, 140, 161, 192));
//        // Irrlicht application draws all 3D objects and all GUI items
//        application.DrawAll();
//
//        application.DoStep();
//        application.EndScene();
//    }
//
//
//
////    //////////////////////////
////
////    ChSystemSMC system;
////
////    auto floor = std::make_shared<ChBodyEasyBox>(10.,1.,10.,1000.);
////    system.Add(floor);
////    floor->SetBodyFixed(true);
////    auto mfloorcolor = std::make_shared<ChColorAsset>();
////    mfloorcolor->SetColor(ChColor(0.3f, 0.3f, 0.6f));
////    floor->AddAsset(mfloorcolor);
////
////
////    auto box = std::make_shared<ChBodyEasyBox>(1.,1.,1.,10.);
////    system.Add(box);
////    box->SetPos(ChVector<>(0.,5.,0.));
////
////    auto mboxcolor = std::make_shared<ChColorAsset>();
////    mboxcolor->SetColor(ChColor(0.6f, 0.3f, 0.3f));
////    box->AddAsset(mboxcolor);
////
////    auto constraint = std::make_shared<ChLinkMatePlane>();
//////    constraint->Initialize(box, floor, true, ChVector<>(0,-.5,0), ChVector<>(0,.5,0), ChVector<>(0,-1,0), ChVector<>(0,1,0));
////    constraint->Initialize(floor, box, false, ChVector<>(0,.5,0), ChVector<>(0,4.5,0), ChVector<>(0,1,0), ChVector<>(0,-1,0));
////    system.AddLink(constraint);
////
////    ChIrrApp application(&system, L"PlaneOnPlane test", core::dimension2d<u32>(800, 600), false, true);
////
////    // Easy shortcuts to add camera, lights, logo and sky in Irrlicht scene:
////    ChIrrWizard::add_typical_Logo(application.GetDevice());
////    ChIrrWizard::add_typical_Sky(application.GetDevice());
////    ChIrrWizard::add_typical_Lights(application.GetDevice());
////    ChIrrWizard::add_typical_Camera(application.GetDevice(), core::vector3df(0, 0, -6));
////    // Bind assets
////    application.AssetBindAll();
////    application.AssetUpdateAll();
////
//////    double fan_radius = 5.3;
//////    IAnimatedMesh* fanMesh = application.GetSceneManager()->getMesh(GetChronoDataFile("fan2.obj").c_str());
//////    IAnimatedMeshSceneNode* fanNode = application.GetSceneManager()->addAnimatedMeshSceneNode(fanMesh);
//////    fanNode->setScale(irr::core::vector3df((irr::f32)fan_radius, (irr::f32)fan_radius, (irr::f32)fan_radius));
////
////    while (application.GetDevice()->run()) {
////        // Irrlicht must prepare frame to draw
////        application.BeginScene(true, true, SColor(255, 140, 161, 192));
////        // Irrlicht application draws all 3D objects and all GUI items
////        application.DrawAll();
////        // Draw also a grid on the horizontal XZ plane
////        ChIrrTools::drawGrid(application.GetVideoDriver(), 2, 2, 20, 20,
////                             ChCoordsys<>(ChVector<>(0, -20, 0), Q_from_AngX(CH_C_PI_2)),
////                             video::SColor(255, 80, 100, 100), true);
////        // Update the position of the spinning fan (an Irrlicht
////        // node, which is here just for aesthetical reasons!)
//////        ChQuaternion<> my_fan_rotation;
//////        my_fan_rotation.Q_from_AngY(system.GetChTime() * -0.5);
//////        ChQuaternion<> my_fan_spin;
//////        my_fan_spin.Q_from_AngZ(system.GetChTime() * 4);
//////        ChCoordsys<> my_fan_coord(ChVector<>(12, -6, 0), my_fan_rotation);
//////        ChFrame<> my_fan_framerotation(my_fan_coord);
//////        ChFrame<> my_fan_framespin(ChCoordsys<>(VNULL, my_fan_spin));
//////        ChCoordsys<> my_fan_coordsys = (my_fan_framespin >> my_fan_framerotation).GetCoord();
//////        ChIrrTools::alignIrrlichtNodeToChronoCsys(fanNode, my_fan_coordsys);
////        // Apply forces caused by fan & wind if Chrono rigid bodies are
////        // in front of the fan, using a simple tutorial function (see above):
//////        apply_fan_force(&system, my_fan_coord, fan_radius, 5.2, 0.5);
////        // HERE CHRONO INTEGRATION IS PERFORMED: THE
////        // TIME OF THE SIMULATION ADVANCES FOR A SINGLE
////        // STEP:
////        application.DoStep();
////        application.EndScene();
////    }
//
//}
