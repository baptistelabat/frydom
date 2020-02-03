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


void makeBox(const::std::shared_ptr<FrBody>& body, double xSize, double ySize, double zSize, double mass) {

    // Properties of the box
    double xSize2 = xSize * xSize;
    double ySize2 = ySize * ySize;
    double zSize2 = zSize * zSize;

    // inertia
    double Ixx = (1./12.) * mass * (ySize2 + zSize2);
    double Iyy = (1./12.) * mass * (xSize2 + zSize2);
    double Izz = (1./12.) * mass * (xSize2 + ySize2);
    double Ixy = 0, Ixz = 0, Iyz = 0;

    Position COG(0.5*xSize, 0.5*ySize, 0.5*zSize);

    FrInertiaTensor inertiaAtCOG(mass, Ixx, Iyy, Izz, 0., 0., 0., COG, NWU);

//    FrFrame corner(Position(-0.5*xSize, -0.5*ySize, -0.5*zSize), FrRotation(), NWU);
//
//    inertiaAtCOG.GetInertiaCoeffsAtFrame(Ixx, Iyy, Izz, Ixy, Ixz, Iyz, corner, NWU);
//
//    FrInertiaTensor inertiaAtCorner (mass, Ixx, Iyy, Izz, Ixy, Ixz, Iyz, -corner.GetPosition(NWU), NWU);

    // Building the chrono body
    body->SetInertiaTensor(inertiaAtCOG);


    // Asset
    body->AddBoxShape(xSize, ySize, zSize);


    for (const auto &box:body->GetBoxShapes()) {
        box->Translate(COG);
    }

}

void AttachBodies(const std::shared_ptr<FrBody>& body1, const std::shared_ptr<FrBody>& body2,
        const Position& Pos1, const Position& Pos2, FRAME_CONVENTION fc) {

    auto nodeName = "Node_" + std::to_string(body1->GetNodeList().size());
    auto thisNode = body1->NewNode(nodeName);
    thisNode->SetPositionInBody(Pos1, fc);

    nodeName = "Node_" + std::to_string(body2->GetNodeList().size());
    auto newNode = body2->NewNode(nodeName);
    newNode->SetPositionInBody(Pos2, fc);

    auto linkName = "FixedLink_" + body1->GetName() + "_" + body2->GetName();
    auto fixedLink = make_fixed_link(linkName, body1->GetSystem(), thisNode, newNode);

}

/// Attach two bodies, with a fixed link
/// \param frame1 frame in body1 reference frame, of the fixed link marker
/// \param frame2 frame in body2 reference frame, of the fixed link marker
/// \return new body created
void AttachBodies(const std::shared_ptr<FrBody>& body1, const std::shared_ptr<FrBody>& body2,
                  const FrFrame& frame1, const FrFrame& frame2) {

    auto nodeName = "Node_" + std::to_string(body1->GetNodeList().size());
    auto thisNode = body1->NewNode(nodeName);
    thisNode->SetFrameInBody(frame1);

    nodeName = "Node_" + std::to_string(body2->GetNodeList().size());
    auto newNode = body2->NewNode(nodeName);
    newNode->SetFrameInBody(frame2);

    auto linkName = "FixedLink_" + body1->GetName() + "_" + body2->GetName();
    auto fixedLink = make_fixed_link(linkName, body1->GetSystem(), thisNode, newNode);

}


TEST(FrAssemblyTest,Add) {
    FRAME_CONVENTION fc = NWU;

    FrOffshoreSystem system("unit_test_FrAssembly_Add");

    system.GetEnvironment()->ShowSeabed(false);
    system.GetEnvironment()->ShowFreeSurface(false);


    double L1 = 10, L2 = 15, L3 = 17;
    double d1 = 0.35, d2 = 0.15, d3 = 0.75;

    auto body1 = system.NewBody("base");
    makeItBox(body1, L1, L2, L3*d3, L1*L2*L3*d3);
    body1->SetColor(DarkRed);
    body1->SetFixedInWorld(true);

    FrAssembly assembly(body1);

//    std::cout<<body1->GetInertiaTensor()<<std::endl;

    if (d1!=0 && d2!=0) {
        auto body2 = system.NewBody("body_2");
        makeItBox(body2, d1 * L1, d2 * L2, (1. - d3) * L3, d1 * L1 * d2 * L2 * (1. - d3) * L3);
        body2->SetColor(DarkBlue);
        AttachBodies(body1, body2, Position(-0.5 * L1, -0.5 * L2, 0.5 * d3 * L3),
                     Position(-0.5 * L1 * d1, -0.5 * L2 * d2, -0.5 * L3 * (1. - d3)), fc);
        assembly.AddToAssembly(body2);

//        std::cout<<body2->GetInertiaTensor()<<std::endl;

        if (d1<1) {
            auto body3 = system.NewBody("body_3");
            makeItBox(body3, (1. - d1) * L1, d2 * L2, (1. - d3) * L3, (1. - d1) * L1 * d2 * L2 * (1. - d3) * L3);
            body3->SetColor(DarkGreen);
            AttachBodies(body2, body3, Position(0.5 * d1 * L1, -0.5 * d2 * L2, -0.5 * (1. - d3) * L3),
                         Position(-0.5 * (1. - d1) * L1, -0.5 * d2 * L2, -0.5 * (1. - d3) * L3), fc);
            assembly.AddToAssembly(body3);

//            std::cout<<body3->GetInertiaTensor()<<std::endl;
        }

    }

    auto body4 = system.NewBody("body_4");
    makeItBox(body4, L1, (1.-d2)*L2, (1.-d3)*L3, L1*(1.-d2)*L2*(1.-d3)*L3);
    body4->SetColor(DarkGoldenRod);
    AttachBodies(body1,body4, Position(0.5*L1,0.5*L2,0.5*d3*L3), Position(0.5*L1, 0.5*(1.-d2)*L2, -0.5*(1.-d3)*L3), fc);
    assembly.AddToAssembly(body4);

//    std::cout<<body4->GetInertiaTensor()<<std::endl;

    auto bodyFull = system.NewBody("full_body");
    bodyFull->SetColor(DarkKhaki);
    makeItBox(bodyFull, L1, L2, L3, L1*L2*L3);
    bodyFull->SetFixedInWorld(true);

    system.SetTimeStep(0.01);

    system.Initialize();
    system.DoAssembly();
//    system.Visualize(30);

//    system.RunInViewer(0.,30);

//    std::cout<<assembly.GetInertiaTensor()<<std::endl;
//    std::cout<<bodyFull->GetInertiaTensor()<<std::endl;


    EXPECT_NEAR(assembly.GetInertiaTensor().GetMass(), bodyFull->GetInertiaTensor().GetMass(), 1e-08);

//    Position testPos = assembly.GetInertiaTensor().GetCOGPosition(fc) - bodyFull->GetInertiaTensor().GetCOGPosition(fc);
//    EXPECT_NEAR(testPos.norm(), 0., 1e-08);

    mathutils::Matrix33<double> testMat = assembly.GetInertiaTensor().GetInertiaMatrixAtCOG(fc) - bodyFull->GetInertiaTensor().GetInertiaMatrixAtCOG(fc);
    EXPECT_NEAR(testMat.norm(), 0., 1e-08);
}



TEST(FrAssemblyTest,AddRotation) {
    FRAME_CONVENTION fc = NWU;

    FrOffshoreSystem system("unit_test_FrAssembly_AddRotation");

    system.GetEnvironment()->ShowSeabed(false);
    system.GetEnvironment()->ShowFreeSurface(false);

    double L1 = 10, L2 = 15, L3 = 17;
    double d1 = 0.35, d2 = 0.15, d3 = 0.75;

    auto body1 = system.NewBody("base");
    makeItBox(body1, L1, L2, L3*d3, L1*L2*L3*d3);
    body1->SetColor(DarkRed);
    body1->SetFixedInWorld(true);

    FrAssembly assembly(body1);

//    std::cout<<body1->GetInertiaTensor()<<std::endl;

    if (d1!=0 && d2!=0) {
        auto body2 = system.NewBody("body_2");
        makeItBox(body2, (1. - d3) * L3, d2 * L2, d1 * L1, d1 * L1 * d2 * L2 * (1. - d3) * L3);
        body2->SetColor(DarkBlue);

        FrFrame body1Frame(Position(-0.5 * L1, -0.5 * L2, 0.5 * d3 * L3), FrRotation(), fc);
        FrFrame body2Frame(Position(-0.5 * L3 * (1. - d3), -0.5 * L2 * d2, 0.5 * L1 * d1), FrRotation(Direction(0.,1.,0), 90*DEG2RAD, fc), fc);

        AttachBodies(body1, body2, body1Frame,body2Frame);
        assembly.AddToAssembly(body2);

//        std::cout<<body2->GetInertiaTensor()<<std::endl;

        if (d1<1) {
            auto body3 = system.NewBody("body_3");
            makeItBox(body3, (1. - d1) * L1, d2 * L2, (1. - d3) * L3, (1. - d1) * L1 * d2 * L2 * (1. - d3) * L3);
            body3->SetColor(DarkGreen);

            FrFrame body2Frame(Position(-0.5 * (1. - d3) * L3, -0.5 * d2 * L2, -0.5 * d1 * L1), FrRotation(Direction(0.,1.,0), 90*DEG2RAD, fc), fc);
            FrFrame body3Frame(Position(-0.5 * (1. - d1) * L1, -0.5 * d2 * L2, -0.5 * (1. - d3) * L3), FrRotation(), fc);

            AttachBodies(body2, body3, body2Frame, body3Frame);
            assembly.AddToAssembly(body3);

//            std::cout<<body3->GetInertiaTensor()<<std::endl;
        }

    }

    auto body4 = system.NewBody("body_4");
    makeItBox(body4, L1, (1.-d3)*L3, (1.-d2)*L2, L1*(1.-d2)*L2*(1.-d3)*L3);
    body4->SetColor(DarkGoldenRod);

    FrFrame body1Frame(Position(0.5*L1,0.5*L2,0.5*d3*L3), FrRotation(), fc);
    FrFrame body4Frame(Position(0.5*L1, -0.5*(1.-d3)*L3, -0.5*(1.-d2)*L2), FrRotation(Direction(1.,0.,0), -90*DEG2RAD, fc), fc);

    AttachBodies(body1, body4, body1Frame, body4Frame);
    assembly.AddToAssembly(body4);

//    std::cout<<body4->GetInertiaTensor()<<std::endl;

    auto bodyFull = system.NewBody("full_body");
    bodyFull->SetColor(DarkKhaki);
    makeItBox(bodyFull, L1, L2, L3, L1*L2*L3);
    bodyFull->SetFixedInWorld(true);

    system.SetTimeStep(0.01);

    system.Initialize();
    system.DoAssembly();
//    system.Visualize(30);

//    system.RunInViewer(0.,30);

//    std::cout<<assembly.GetInertiaTensor()<<std::endl;
//    std::cout<<bodyFull->GetInertiaTensor()<<std::endl;


    EXPECT_NEAR(assembly.GetInertiaTensor().GetMass(), bodyFull->GetInertiaTensor().GetMass(), 1e-08);

//    Position testPos = assembly.GetInertiaTensor().GetCOGPosition(fc) - bodyFull->GetInertiaTensor().GetCOGPosition(fc);
//    EXPECT_NEAR(testPos.norm(), 0., 1e-08);

    mathutils::Matrix33<double> testMat = assembly.GetInertiaTensor().GetInertiaMatrixAtCOG(fc) - bodyFull->GetInertiaTensor().GetInertiaMatrixAtCOG(fc);
    EXPECT_NEAR(testMat.norm(), 0., 1e-08);

}



TEST(FrAssemblyTest,AddCOG) {
    FRAME_CONVENTION fc = NWU;

    FrOffshoreSystem system("unit_test_FrAssembly_AddCOG");

    system.GetEnvironment()->ShowSeabed(false);
    system.GetEnvironment()->ShowFreeSurface(false);

    double L1 = 10, L2 = 15, L3 = 17;
    double d1 = 0.35, d2 = 0.15, d3 = 0.75;

    auto body1 = system.NewBody("base");
    makeBox(body1, L1, L2, L3*d3, L1*L2*L3*d3);
    body1->SetColor(DarkRed);
    body1->SetFixedInWorld(true);

    FrAssembly assembly(body1);

//    std::cout<<body1->GetInertiaTensor()<<std::endl;

    if (d1!=0 && d2!=0) {
        auto body2 = system.NewBody("body_2");
        makeBox(body2, d1 * L1, d2 * L2, (1. - d3) * L3, d1 * L1 * d2 * L2 * (1. - d3) * L3);
        body2->SetColor(DarkBlue);
        AttachBodies(body1, body2, Position(0., 0., d3 * L3),
                     Position(0., 0., 0.), fc);
        assembly.AddToAssembly(body2);

//        std::cout<<body2->GetInertiaTensor()<<std::endl;

        if (d1<1) {
            auto body3 = system.NewBody("body_3");
            makeBox(body3, (1. - d1) * L1, d2 * L2, (1. - d3) * L3, (1. - d1) * L1 * d2 * L2 * (1. - d3) * L3);
            body3->SetColor(DarkGreen);
            AttachBodies(body2, body3, Position(d1 * L1, 0., 0.),
                         Position(0., 0., 0.), fc);
            assembly.AddToAssembly(body3);

//            std::cout<<body3->GetInertiaTensor()<<std::endl;
        }

    }

    auto body4 = system.NewBody("body_4");
    makeBox(body4, L1, (1.-d2)*L2, (1.-d3)*L3, L1*(1.-d2)*L2*(1.-d3)*L3);
    body4->SetColor(DarkGoldenRod);
    AttachBodies(body1,body4, Position(L1,L2,d3*L3), Position(L1, (1.-d2)*L2, 0.), fc);
    assembly.AddToAssembly(body4);

//    std::cout<<body4->GetInertiaTensor()<<std::endl;

    auto bodyFull = system.NewBody("full_body");
    bodyFull->SetColor(DarkKhaki);
    makeBox(bodyFull, L1, L2, L3, L1*L2*L3);
    bodyFull->SetFixedInWorld(true);

    system.SetTimeStep(0.01);

    system.Initialize();
    system.DoAssembly();
//    system.Visualize(30);

//    system.RunInViewer(0.,30);

//    std::cout<<assembly.GetInertiaTensor()<<std::endl;
//    std::cout<<bodyFull->GetInertiaTensor()<<std::endl;


    EXPECT_NEAR(assembly.GetInertiaTensor().GetMass(), bodyFull->GetInertiaTensor().GetMass(), 1e-08);

    Position testPos = assembly.GetInertiaTensor().GetCOGPosition(fc) - bodyFull->GetInertiaTensor().GetCOGPosition(fc);
    EXPECT_NEAR(testPos.norm(), 0., 1e-08);

    mathutils::Matrix33<double> testMat = assembly.GetInertiaTensor().GetInertiaMatrixAtCOG(fc) - bodyFull->GetInertiaTensor().GetInertiaMatrixAtCOG(fc);
    EXPECT_NEAR(testMat.norm(), 0., 1e-08);

}


TEST(FrAssemblyTest,AddCOGRotation) {
    FRAME_CONVENTION fc = NWU;

    FrOffshoreSystem system("unit_test_FrAssembly_AddCOGRotation");

    system.GetEnvironment()->ShowSeabed(false);
    system.GetEnvironment()->ShowFreeSurface(false);

    double L1 = 10, L2 = 15, L3 = 17;
    double d1 = 0.35, d2 = 0.15, d3 = 0.75;

    auto body1 = system.NewBody("base");
    makeBox(body1, L1, L2, L3*d3, L1*L2*L3*d3);
    body1->SetColor(DarkRed);
    body1->SetFixedInWorld(true);

    FrAssembly assembly(body1);

//    std::cout<<body1->GetInertiaTensor()<<std::endl;

    if (d1!=0 && d2!=0) {
        auto body2 = system.NewBody("body_2");
        makeBox(body2, (1. - d3) * L3, d2 * L2, d1 * L1, d1 * L1 * d2 * L2 * (1. - d3) * L3);
        body2->SetColor(DarkBlue);

        FrFrame body1Frame(Position(0., 0., d3 * L3), FrRotation(), fc);
        FrFrame body2Frame(Position(0., 0., L1 * d1), FrRotation(Direction(0.,1.,0), 90*DEG2RAD, fc), fc);

        AttachBodies(body1, body2, body1Frame,body2Frame);
        assembly.AddToAssembly(body2);

//        std::cout<<body2->GetInertiaTensor()<<std::endl;

        if (d1<1) {
            auto body3 = system.NewBody("body_3");
            makeBox(body3, (1. - d1) * L1, d2 * L2, (1. - d3) * L3, (1. - d1) * L1 * d2 * L2 * (1. - d3) * L3);
            body3->SetColor(DarkGreen);

            FrFrame body2Frame(Position(0., 0., 0.), FrRotation(Direction(0.,1.,0), 90*DEG2RAD, fc), fc);
            FrFrame body3Frame(Position(0., 0., 0.), FrRotation(), fc);

            AttachBodies(body2, body3, body2Frame, body3Frame);
            assembly.AddToAssembly(body3);

//            std::cout<<body3->GetInertiaTensor()<<std::endl;
        }

    }

    auto body4 = system.NewBody("body_4");
    makeBox(body4, L1, (1.-d3)*L3, (1.-d2)*L2, L1*(1.-d2)*L2*(1.-d3)*L3);
    body4->SetColor(DarkGoldenRod);

    FrFrame body1Frame(Position(L1,L2,d3*L3), FrRotation(), fc);
    FrFrame body4Frame(Position(L1, 0., 0.), FrRotation(Direction(1.,0.,0), -90*DEG2RAD, fc), fc);

    AttachBodies(body1, body4, body1Frame, body4Frame);
    assembly.AddToAssembly(body4);

//    std::cout<<body4->GetInertiaTensor()<<std::endl;


    auto bodyFull = system.NewBody("full_body");
    bodyFull->SetColor(DarkKhaki);
    makeBox(bodyFull, L1, L2, L3, L1*L2*L3);
    bodyFull->SetFixedInWorld(true);

    system.SetTimeStep(0.01);

    system.Initialize();
    system.DoAssembly();
//    system.Visualize(30);

//    system.RunInViewer(0.,30);

//    std::cout<<assembly.GetInertiaTensor()<<std::endl;
//    std::cout<<bodyFull->GetInertiaTensor()<<std::endl;

    EXPECT_NEAR(assembly.GetInertiaTensor().GetMass(), bodyFull->GetInertiaTensor().GetMass(), 1e-08);

    Position testPos = assembly.GetInertiaTensor().GetCOGPosition(fc) - bodyFull->GetInertiaTensor().GetCOGPosition(fc);
    EXPECT_NEAR(testPos.norm(), 0., 1e-08);

    mathutils::Matrix33<double> testMat = assembly.GetInertiaTensor().GetInertiaMatrixAtCOG(fc) - bodyFull->GetInertiaTensor().GetInertiaMatrixAtCOG(fc);
    EXPECT_NEAR(testMat.norm(), 0., 1e-08);

}
