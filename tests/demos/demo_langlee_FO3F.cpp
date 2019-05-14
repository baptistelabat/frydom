//
// Created by camille on 12/04/19.
//

#include "frydom/frydom.h"

using namespace frydom;

int main(int argc, char* argv[]) {

    std::cout << " ====================================== Demo Langlee F3OF ========================== " << std::endl;

    // System

    FrOffshoreSystem system;

    // Environment

    auto ocean = system.GetEnvironment()->GetOcean();
    auto waveField = ocean->GetFreeSurface()->SetAiryRegularWaveField();
    waveField->SetWaveHeight(0.);
    waveField->SetWavePeriod(10.);
    waveField->SetDirection(0., DEG, NWU, GOTO);

    system.GetEnvironment()->GetTimeRamp()->SetByTwoPoints(0., 0., 15., 1.);
    system.GetEnvironment()->GetTimeRamp()->SetActive(false);

    //ocean->GetFreeSurface()->GetFreeSurfaceGridAsset()->SetGrid(-14., 14, 2, -14, 14, 2);

    // Bodies

    auto barge = system.NewBody();
    barge->SetName("barge");
    barge->AddMeshAsset("Barge.obj");
    barge->SetPosition(Position(0., 0., -9.), NWU);

    double mass_b = 1.089825e6;
    double Iy_b = 7.63e7;
    FrFrame COGFrame(Position(0., 0., 0.), FrRotation(), NWU);
    FrInertiaTensor InertiaTensor(mass_b, 1., Iy_b, 1., 0., 0., 0., COGFrame, NWU);

    barge->SetInertiaTensor(InertiaTensor);

    barge->GetDOFMask()->MakeItLocked();

    auto node_1b = barge->NewNode();
    node_1b->SetPositionInBody(Position(-12.5, 0., 0.), NWU);
    node_1b->RotateAroundXInBody(-90.*DEG2RAD, NWU);

    auto node_2b = barge->NewNode();
    node_2b->SetPositionInBody(Position(12.5, 0., 0.), NWU);
    node_2b->RotateAroundXInBody(-90.*DEG2RAD, NWU);

    // Flap1

    auto flap1 = system.NewBody();
    flap1->SetName("flap1");
    flap1->AddMeshAsset("Flap1_fuse.obj");
    flap1->SetPosition(Position(-12.5, 0., -9.), NWU);

    double mass_f1 = 1.7925e5;
    double Iy_f1 = 1.3e6;
    FrFrame COGFrame_flap(Position(0., 0., 3.5), FrRotation(), NWU);
    FrInertiaTensor InertiaTensor_flap(mass_f1, 1., Iy_f1, 1., 0., 0., 0., COGFrame_flap, NWU);

    flap1->SetInertiaTensor(InertiaTensor_flap);

    auto node_1f = flap1->NewNode();
    node_1f->SetPositionInBody(Position(0., 0., 0), NWU);
    node_1f->RotateAroundXInBody(-90.*DEG2RAD, NWU);

    auto rev1 = make_revolute_link(node_1f, node_1b, &system);

    // Flap2

    auto flap2 = system.NewBody();
    flap2->SetName("flap2");
    flap2->AddMeshAsset("Flap2_fuse.obj");
    flap2->SetPosition(Position(12.5, 0., -9.), NWU);

    flap2->SetInertiaTensor(InertiaTensor_flap);

    auto node_2f = flap2->NewNode();
    node_2f->SetPositionInBody(Position(0., 0., 0), NWU);
    node_2f->RotateAroundXInBody(-90.*DEG2RAD, NWU);

    auto rev2 = make_revolute_link(node_2f, node_2b, &system);

    // Hydrodynamic

    auto hdb = make_hydrodynamic_database("Langlee.hdb5");

    auto eqFrame0 = std::make_shared<FrEquilibriumFrame>(barge.get());
    auto eqFrame1 = std::make_shared<FrEquilibriumFrame>(flap1.get());
    auto eqFrame2 = std::make_shared<FrEquilibriumFrame>(flap2.get());

    hdb->Map(0, flap1.get(), eqFrame1);
    hdb->Map(1, flap2.get(), eqFrame2);
    hdb->Map(2, barge.get(), eqFrame0);

    auto radiationModel = make_radiation_convolution_model(hdb, &system);
    radiationModel->SetImpulseResponseSize(flap1.get(), 60., 0.005);
    radiationModel->SetImpulseResponseSize(flap2.get(), 60., 0.005);
    radiationModel->SetImpulseResponseSize(barge.get(), 60., 0.005);

    // Hydrostatic

    auto forceHst1 = make_linear_hydrostatic_force(hdb, flap1);
    auto forceHst2 = make_linear_hydrostatic_force(hdb, flap2);

    // Excitation

    auto excitation1 = make_linear_excitation_force(hdb, flap1);
    auto excitation2 = make_linear_excitation_force(hdb, flap2);

    // Simulation

    auto dt = 0.005;

    system.SetTimeStep(dt);

    system.Initialize();

    flap1->Rotate(FrRotation(Direction(0,1,0), -10.*DEG2RAD, NWU));

    //system.RunInViewer(0, 50, false);
    //system.Visualize(50, false);

    auto time = 0.;

    while (time < 200.) {

        time += dt;

        system.AdvanceTo(time);

        std::cout << "Time : " << time << " s" << std::endl;

    }

    std::cout << " ==================================== End ========================== " << std::endl;

    return 0;

}

