//
// Created by lletourn on 06/06/19.
//

#include "frydom/frydom.h"

using namespace frydom;


int main() {

    FRAME_CONVENTION fc = NWU;

    FrOffshoreSystem system;

    //-------------------------------------
    // Barge
    //-------------------------------------

    auto CB28 = system.NewBody();
    CB28->SetName("CB28");
    CB28->AddMeshAsset("CB28_Full.obj");
    CB28->SetColor(Yellow);
    CB28->AllowCollision(false);

    // Inertia
    double mass = 1E6;
    Position COG = {12.,6.,13.44};
    FrFrame COGFrame(COG, FrRotation(), fc);

    double Ixx = 7.9E7, Iyy = 7.7E8, Izz = 8.5E8;

    CB28->SetInertiaTensor(FrInertiaTensor(mass,Ixx,Iyy,Izz,0.,0.,0.,COG,fc));

    auto eqFrame = std::make_shared<FrEquilibriumFrame>(CB28.get());
    system.AddPhysicsItem(eqFrame);

    // hydrostatic
    double K33 = 1.8E7, K44 = 9.2E8, K55 = 9E9;
    FrLinearHydrostaticStiffnessMatrix hydrostaticStiffness;
    hydrostaticStiffness.SetDiagonal(K33, K44, K55);

    auto hydrostaticForce = make_linear_hydrostatic_force(eqFrame, CB28);
    hydrostaticForce->SetStiffnessMatrix(hydrostaticStiffness);


    //-------------------------------------
    // Pile
    //-------------------------------------

    FrFrame pileFrame(Position(0.,-8.,0.), FrRotation(), fc);
    pileFrame.RotX_DEGREES(-90,fc,true);

    FrFrame CB28Frame(Position(7.6, -4.064, 4.04), FrRotation(), fc);

    auto pile = CB28->NewBody(CB28Frame, pileFrame);
    pile->SetName("Pile");
    pile->SetColor(DarkRed);
    makeItCylinder(pile, 3, 16, 80E3);
    pile->AllowCollision(false);

    //-------------------------------------
    // Manifold
    //-------------------------------------

    auto manifold = CB28->NewBody(Position(-8, -5.6, 4.04), Position(0.,0.,-3), fc);
    manifold->SetName("Manifold");
    manifold->SetColor(DarkGreen);
    makeItBox(manifold, 16, 9.5, 6, 140E3);
    manifold->AllowCollision(false);

//    system.Initialize();
//    system.DoAssembly();

    system.SetTimeStep(0.01);

//    system.SolveStaticWithRelaxation();

    system.RunInViewer(0., 50, false);
//    system.Visualize(50, false);
}
