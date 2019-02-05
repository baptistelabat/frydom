//
// Created by frongere on 25/01/19.
//

#include "frydom/frydom.h"

#include "gtest/gtest.h"

using namespace frydom;


int main() {

    FrOffshoreSystem_ system;
//    system.SetGravityAcceleration(1);
    system.GetEnvironment()->ShowFreeSurface(false);


    // Body 1 definition (fixed body)
    auto body1 = system.NewBody();
//    body1->SetFixedInWorld(true);
    makeItBox(body1, 20, 10, 2, 1000);
    body1->AllowCollision(false);
    body1->SetColor(MediumVioletRed);

    auto node1 = body1->NewNode();
    node1->TranslateInBody(-10, 0, 0, NWU);
    node1->RotateAroundXInBody(90*DEG2RAD, NWU);

    auto nodeWorld = system.GetWorldBody()->NewNode();
    nodeWorld->TranslateInWorld(-10, 0, 0, NWU);
    nodeWorld->RotateAroundXInBody(90*DEG2RAD, NWU);

    auto rev1 = make_revolute_link(node1, nodeWorld, &system);



    // Body 2 definition (linked body)
    auto body2 = system.NewBody();
    makeItBox(body2, 2, 2, 40, 2000);
    body2->SetColor(Black);
    body2->TranslateInWorld(10, 5, 0, NWU);
//    body2->SetFixedInWorld(true);

//    body2->SetRotation(FrRotation_(Direction(0, 1, 0), 90*DEG2RAD, NWU));

//    std::cout << body1->GetCOG(NWU) << body1->GetPosition(NWU) << std::endl;


    auto m1 = body1->NewNode();
    m1->TranslateInBody(10, 5, -1, NWU);


    auto m2 = body2->NewNode();
    m2->TranslateInBody(-1, -1, -20, NWU);

    auto prismaticLink = make_prismatic_link(m1, m2, &system);
    prismaticLink->SetSpringDamper(2e3, 1e3);
    prismaticLink->SetRestLength(-5);



    auto body3 = system.NewBody();
    makeItBox(body3, 2, 2, 6, 500);
    body3->AllowCollision(false);
    body3->SetColor(Red);
//    body3->Rotate(FrRotation_(Direction(0, 1, 0), 90*DEG2RAD, NWU));
    body3->TranslateInWorld(-10, 5, -1, NWU);
//    body3->TranslateInWorld(0, 5, -1, NWU);
    body3->SetMaxRotationSpeed(1*RAD2DEG);
    body3->ActivateSpeedLimits(true);


    auto m3 = body1->NewNode();
    m3->TranslateInBody(-10, 5, -1, NWU);
//    m3->RotateAroundYInBody(90*DEG2RAD, NWU);

    auto m4 = body3->NewNode();
//    m4->TranslateInBody()


    auto revoluteLink = make_revolute_link(m3, m4, &system);
    revoluteLink->SetSpringDamper(1e4, 1e1);
    revoluteLink->SetRestAngle(180*DEG2RAD);




    system.SetTimeStep(0.01);
    system.RunInViewer(0, 50, true);



    /*
     *
     * implementer les methodes permettant de recuperer les valeurs des ddl de la liaison
     *
     * ajouter maintenant des raideurs et amortissement dans la liaison
     *
     * Implementer le calculde la force dans la liaison
     *
     * Implementer la methode de calcul de la reaction
     *
     * Ajouter methode disant MakeThisConfigurationAsReference
     *
     * SetSpringDamping(double stiffness, double damping, double restLength);
     *
     *
     *
     *
     */





    return 0;
}

