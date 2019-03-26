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

#include "frydom/frydom.h"
//#include "gtest/gtest.h"

using namespace frydom;



//--------------- Links static analysis --------------------------//

int link_static_analysis() {

    FrOffshoreSystem system;
    system.GetEnvironment()->ShowFreeSurface(false);

    // Body1 definition
    auto body1 = system.NewBody();
    body1->SetName("1");
    makeItBox(body1, 20, 10, 2, 1000);
    body1->AllowCollision(false);
    body1->SetColor(MediumVioletRed);


    // Revolute link between body1 and world
    auto node1 = body1->NewNode();
    node1->TranslateInBody(-10, 0, 0, NWU);
    node1->RotateAroundXInBody(90*DEG2RAD, NWU);

    auto nodeWorld = system.GetWorldBody()->NewNode();
    nodeWorld->TranslateInWorld(-10, 0, 0, NWU);
    nodeWorld->RotateAroundXInBody(90*DEG2RAD, NWU);

    auto rev1 = make_revolute_link(node1, nodeWorld, &system);
//    rev1->SetSpringDamper(5e5, 1e1);
//    rev1->SetRestAngle(0*DEG2RAD);


    // Body 2 definition (linked body)
    auto body2 = system.NewBody();
    body2->SetName("2");
    makeItBox(body2, 2, 2, 40, 2000);
    body2->SetColor(Black);
    body2->TranslateInWorld(10, 5, 0, NWU);

    // Prismatic link between body1 and body2
    auto m1 = body1->NewNode();
    m1->TranslateInBody(10, 5, -1, NWU);

    auto m2 = body2->NewNode();
    m2->TranslateInBody(-1, -1, -20, NWU);

    auto prismaticLink = make_prismatic_link(m1, m2, &system);
    prismaticLink->SetSpringDamper(2e3, 1e3);
    prismaticLink->SetRestLength(-5);




    // Body 3 definition
    auto body3 = system.NewBody();
    body3->SetName("3");
    makeItBox(body3, 2, 2, 6, 500);
    body3->AllowCollision(false);
    body3->SetColor(Red);
    body3->TranslateInWorld(-10, 5, -1, NWU);

    // Revolute link between body1 and body3
    auto m3 = body1->NewNode();
    m3->TranslateInBody(-10, 5, -1, NWU);

    auto m4 = body3->NewNode();

    auto revoluteLink = make_revolute_link(m3, m4, &system);
    revoluteLink->SetSpringDamper(1e4, 1e1);
    revoluteLink->SetRestAngle(0*180*DEG2RAD);

//    system.Visualize(50., false);


    system.SetTimeStep(0.01);
//    system.RunInViewer(0, 50, false);

//    system.Visualize(50., false);
    system.SetNbStepsStatics(100);
    system.SolveStaticEquilibrium(FrOffshoreSystem::STATICS_METHOD::NONLINEAR);
    system.Visualize(50., false);

    return 0;
}


void HSNL_static_analysis() {

    FrOffshoreSystem system;

    // Mooring buoy definition
    double buoyRadius = 1.5;
    double buoyMass = 7250;
    double buoyDamping = 1000;
    auto buoy = make_mooring_buoy(&system, buoyRadius, buoyMass, true, buoyDamping);

    buoy->SetName("Buoy");
    buoy->SetPosition(Position(0.,0.,-3.), NWU);

//    system.RunInViewer(0, 50, false);

    system.SetNbStepsStatics(100);
    system.SolveStaticEquilibrium(FrOffshoreSystem::STATICS_METHOD::QUASISTATIC);
    system.Visualize(50., false);

}


void Catenary_static_analysis() {

    FrOffshoreSystem system;
    system.GetEnvironment()->ShowFreeSurface(false);

    auto body = system.NewBody();
    makeItBox(body,2.,2.,2.,1000);
    body->SetColor(Red);

    auto bodyNode = body->NewNode();

    // Line properties
    bool elastic = true;
    double unstretchedLength = 10.;
    auto u = Direction(0, 0, -1);
    double linearDensity = 600;
    double EA = 5e7;
    double sectionArea = 0.05;
    double YoungModulus = EA / sectionArea;
    double breakTensionAsset = 500000;

    auto worldNodeE = system.GetWorldBody()->NewNode();
    worldNodeE->SetPositionInBody(Position(0.,-10,0.), NWU);

    auto worldNodeW = system.GetWorldBody()->NewNode();
    worldNodeW->SetPositionInBody(Position(0.,10,0.), NWU);

    auto catenaryE = make_catenary_line(worldNodeE, bodyNode, &system, elastic, YoungModulus, sectionArea, unstretchedLength, linearDensity, WATER);
    auto catenaryW = make_catenary_line(worldNodeW, bodyNode, &system, elastic, YoungModulus, sectionArea, unstretchedLength, linearDensity, WATER);

//    system.RunInViewer(0, 30, false);

    system.SetNbStepsStatics(100);
    system.SolveStaticEquilibrium(FrOffshoreSystem::STATICS_METHOD::QUASISTATIC);
    system.Visualize(50., false);
}

int main() {

    // Static analysis of bodies with links
//    link_static_analysis();

    // Static analysis of a mooring buoy with embedded nonlinear hydrostatic
    HSNL_static_analysis();

    // Static analysis of a body connected to 2 catenary lines to the world body
//    Catenary_static_analysis();
}