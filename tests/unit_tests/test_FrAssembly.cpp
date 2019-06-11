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
#include "gtest/gtest.h"

using namespace frydom;

void AttachBodies(const std::shared_ptr<FrBody>& body1, const std::shared_ptr<FrBody>& body2,
        const Position& Pos1, const Position& Pos2, FRAME_CONVENTION fc) {

    auto thisNode = body1->NewNode();
    thisNode->SetPositionInBody(Pos1, fc);

    auto newNode = body2->NewNode();
    newNode->SetPositionInBody(Pos2, fc);

    auto fixedLink = make_fixed_link(thisNode, newNode, body1->GetSystem());

}

/// Attach two bodies, with a fixed link
/// \param frame1 frame in body1 reference frame, of the fixed link marker
/// \param frame2 frame in body2 reference frame, of the fixed link marker
/// \return new body created
void AttachBodies(const std::shared_ptr<FrBody>& body1, const std::shared_ptr<FrBody>& body2,
                  const FrFrame& frame1, const FrFrame& frame2) {

    auto thisNode = body1->NewNode();
    thisNode->SetFrameInBody(frame1);

    auto newNode = body2->NewNode();
    newNode->SetFrameInBody(frame2);

    auto fixedLink = make_fixed_link(thisNode, newNode, body1->GetSystem());

}


TEST(FrAssemblyTest,Add) {
    FRAME_CONVENTION fc = NWU;

    FrOffshoreSystem system;

    system.GetEnvironment()->ShowSeabed(false);
    system.GetEnvironment()->ShowFreeSurface(false);

    FrAssembly assembly;

    double L1 = 10, L2 = 15, L3 = 17;
    double d1 = 0.35, d2 = 0.15, d3 = 0.75;

    auto body1 = system.NewBody();
    makeItBox(body1, L1, L2, L3*d3, L1*L2*L3*d3);
    body1->SetColor(DarkRed);
    body1->SetFixedInWorld(true);
    assembly.AddToAssembly(body1);

    std::cout<<body1->GetInertiaTensor(fc)<<std::endl;

    if (d1!=0 && d2!=0) {
        auto body2 = system.NewBody();
        makeItBox(body2, d1 * L1, d2 * L2, (1. - d3) * L3, d1 * L1 * d2 * L2 * (1. - d3) * L3);
        body2->SetColor(DarkBlue);
        AttachBodies(body1, body2, Position(-0.5 * L1, -0.5 * L2, 0.5 * d3 * L3),
                     Position(-0.5 * L1 * d1, -0.5 * L2 * d2, -0.5 * L3 * (1. - d3)), fc);
        assembly.AddToAssembly(body2);

        std::cout<<body2->GetInertiaTensor(fc)<<std::endl;

        if (d1<1) {
            auto body3 = system.NewBody();
            makeItBox(body3, (1. - d1) * L1, d2 * L2, (1. - d3) * L3, (1. - d1) * L1 * d2 * L2 * (1. - d3) * L3);
            body3->SetColor(DarkGreen);
            AttachBodies(body2, body3, Position(0.5 * d1 * L1, -0.5 * d2 * L2, -0.5 * (1. - d3) * L3),
                         Position(-0.5 * (1. - d1) * L1, -0.5 * d2 * L2, -0.5 * (1. - d3) * L3), fc);
            assembly.AddToAssembly(body3);
        }

    }

    auto body4 = system.NewBody();
    makeItBox(body4, L1, (1.-d2)*L2, (1.-d3)*L3, L1*(1.-d2)*L2*(1.-d3)*L3);
    body4->SetColor(DarkGoldenRod);
    AttachBodies(body1,body4, Position(0.5*L1,0.5*L2,0.5*d3*L3), Position(0.5*L1, 0.5*(1.-d2)*L2, -0.5*(1.-d3)*L3), fc);
    assembly.AddToAssembly(body4);

    std::cout<<body4->GetInertiaTensor(fc)<<std::endl;

    auto bodyFull = system.NewBody();
    bodyFull->SetColor(DarkKhaki);
    makeItBox(bodyFull, L1, L2, L3, L1*L2*L3);
    bodyFull->SetFixedInWorld(true);
//    bodyFull->SetPosition(Position(2*L1, 0., 0.), fc);

    system.SetTimeStep(0.01);

    system.Initialize();
    system.DoAssembly();

    std::cout<<assembly.GetInertiaTensor(fc)<<std::endl;
    std::cout<<bodyFull->GetInertiaTensor(fc)<<std::endl;

//    system.RunInViewer(0.,30);
    system.Visualize(30);

}



TEST(FrAssemblyTest,AddRotation) {
    FRAME_CONVENTION fc = NWU;

    FrOffshoreSystem system;

    system.GetEnvironment()->ShowSeabed(false);
    system.GetEnvironment()->ShowFreeSurface(false);

    FrAssembly assembly;

    double L1 = 10, L2 = 15, L3 = 17;
    double d1 = 0.35, d2 = 0.15, d3 = 0.75;

    auto body1 = system.NewBody();
    makeItBox(body1, L1, L2, L3*d3, L1*L2*L3*d3);
    body1->SetColor(DarkRed);
    body1->SetFixedInWorld(true);
    assembly.SetMasterBody(body1);

    std::cout<<body1->GetInertiaTensor(fc)<<std::endl;

    if (d1!=0 && d2!=0) {
        auto body2 = system.NewBody();
        makeItBox(body2, (1. - d3) * L3, d2 * L2, d1 * L1, d1 * L1 * d2 * L2 * (1. - d3) * L3);
        body2->SetColor(DarkBlue);

        FrFrame body1Frame(Position(-0.5 * L1, -0.5 * L2, 0.5 * d3 * L3), FrRotation(), fc);
        FrFrame body2Frame(Position(-0.5 * L3 * (1. - d3), -0.5 * L2 * d2, 0.5 * L1 * d1), FrRotation(Direction(0.,1.,0), 90*DEG2RAD, fc), fc);

        AttachBodies(body1, body2, body1Frame,body2Frame);
        assembly.AddToAssembly(body2);

        std::cout<<body2->GetInertiaTensor(fc)<<std::endl;

        if (d1<1) {
            auto body3 = system.NewBody();
            makeItBox(body3, (1. - d1) * L1, d2 * L2, (1. - d3) * L3, (1. - d1) * L1 * d2 * L2 * (1. - d3) * L3);
            body3->SetColor(DarkGreen);


            FrFrame body2Frame(Position(-0.5 * (1. - d3) * L3, -0.5 * d2 * L2, -0.5 * d1 * L1), FrRotation(Direction(0.,1.,0), 90*DEG2RAD, fc), fc);
            FrFrame body3Frame(Position(-0.5 * (1. - d1) * L1, -0.5 * d2 * L2, -0.5 * (1. - d3) * L3), FrRotation(), fc);

            AttachBodies(body2, body3, body2Frame, body3Frame);
            assembly.AddToAssembly(body3);
        }

    }

    auto body4 = system.NewBody();
//    makeItBox(body4, L1, (1.-d2)*L2, (1.-d3)*L3, L1*(1.-d2)*L2*(1.-d3)*L3);
    makeItBox(body4, L1, (1.-d3)*L3, (1.-d2)*L2, L1*(1.-d2)*L2*(1.-d3)*L3);
    body4->SetColor(DarkGoldenRod);

    FrFrame body1Frame(Position(0.5*L1,0.5*L2,0.5*d3*L3), FrRotation(), fc);
    FrFrame body4Frame(Position(0.5*L1, -0.5*(1.-d3)*L3, -0.5*(1.-d2)*L2), FrRotation(Direction(1.,0.,0), -90*DEG2RAD, fc), fc);

    AttachBodies(body1, body4, body1Frame, body4Frame);
    assembly.AddToAssembly(body4);

    std::cout<<body4->GetInertiaTensor(fc)<<std::endl;
//    bodyFull->SetPosition(Position(2*L1, 0., 0.), fc);

    system.SetTimeStep(0.01);

    system.Initialize();
    system.DoAssembly();
    system.Visualize(30);

//    system.RunInViewer(0.,30);

    std::cout<<assembly.GetInertiaTensor(fc)<<std::endl;


    auto bodyFull = system.NewBody();
    bodyFull->SetColor(DarkKhaki);
    makeItBox(bodyFull, L1, L2, L3, L1*L2*L3);
    bodyFull->SetFixedInWorld(true);
    std::cout<<bodyFull->GetInertiaTensor(fc)<<std::endl;

}
