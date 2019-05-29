//
// Created by camille on 09/05/19.
//

#include "frydom/frydom.h"


using namespace frydom;

int main(int argc, char* argv[]) {

    std::cout << " ============================================== Demo FOSWEC ============================= " << std::endl;

    // System

    FrOffshoreSystem system;

    system.GetEnvironment()->GetOcean()->GetFreeSurface()->GetFreeSurfaceGridAsset()->SetGrid(-1, 1, 2, -1, 1, 2);

    // Platform

    auto platform = system.NewBody();
    platform->SetName("platform");
    platform->AddMeshAsset("FullPlatform.obj");

    FrFrame COGFrame_b(Position(0., 0., 0.460), FrRotation(), NWU);
    FrInertiaTensor inertia_b(153.8, 37.88, 29.63, 1., 0., 0., 0., COGFrame_b, NWU);
    platform->SetInertiaTensor(inertia_b);

    platform->GetDOFMask()->MakeItLocked();

    auto node_1b = platform->NewNode();
    node_1b->SetPositionInBody(Position(-0.65, 0., -0.5008), NWU);
    node_1b->RotateAroundXInBody(-90.*DEG2RAD, NWU);

    auto node_2b = platform->NewNode();
    node_2b->SetPositionInBody(Position(0.65, 0., -0.5008), NWU);
    node_2b->RotateAroundXInBody(-90.*DEG2RAD, NWU);

    // Flap1

    auto flap1 = system.NewBody();
    flap1->SetName("flap1");
    flap1->AddMeshAsset("FullFlap_mesh.obj");
    flap1->SetPosition(Position(-0.65, 0., -0.29), NWU);

    FrFrame COGFrame_f1(Position(0., 0., 0.), FrRotation(), NWU);
    FrInertiaTensor inertia_f1(23.1, 1.42, 1.19, 1.99, 0., 0., 0., COGFrame_f1, NWU);
    flap1->SetInertiaTensor(inertia_f1);

    auto node_1f = flap1->NewNode();
    node_1f->SetPositionInBody(Position(0., 0., -0.2108), NWU);
    node_1f->RotateAroundXInBody(-90.*DEG2RAD, NWU);

    auto rev1 = make_revolute_link(node_1f, node_1b, &system);

    // Flap2

    auto flap2 = system.NewBody();
    flap2->SetName("flap2");
    flap2->AddMeshAsset("FullFlap_mesh.obj");
    flap2->SetPosition(Position(0.65, 0., -0.29), NWU);

    FrFrame COGFrame_f2(Position(0., 0., 0.), FrRotation(), NWU);
    FrInertiaTensor inertia_f2(23.1, 1.42, 1.19, 1.99, 0., 0., 0., COGFrame_f2, NWU);
    flap2->SetInertiaTensor(inertia_f2);

    auto node_2f = flap2->NewNode();
    node_2f->SetPositionInBody(Position(0., 0., -0.2108), NWU);
    node_2f->RotateAroundXInBody(-90.*DEG2RAD, NWU);

    auto rev2 = make_revolute_link(node_2f, node_2b, &system);

    // Hydrodynamic

    auto hdb = make_hydrodynamic_database("FOSWEC.hdb5");

    auto eqFrame0 = std::make_shared<FrEquilibriumFrame>(platform.get());
    auto eqFrame1 = std::make_shared<FrEquilibriumFrame>(flap1.get());
    auto eqFrame2 = std::make_shared<FrEquilibriumFrame>(flap2.get());

    hdb->Map(0, flap1.get(), eqFrame1);
    hdb->Map(1, flap2.get(), eqFrame2);
    hdb->Map(2, platform.get(), eqFrame0);

    auto radiationModel = make_radiation_convolution_model(hdb, &system);
    radiationModel->SetImpulseResponseSize(flap1.get(), 60., 0.005);
    radiationModel->SetImpulseResponseSize(flap2.get(), 60., 0.005);
    radiationModel->SetImpulseResponseSize(platform.get(), 60., 0.005);

    // Hydrostatic

    auto forceHst1 = make_linear_hydrostatic_force(hdb, flap1);
    auto forceHst2 = make_linear_hydrostatic_force(hdb, flap2);

    // Excitation

    //auto excitation1 = make_linear_excitation_force(hdb, flap1);
    //auto excitation2 = make_linear_excitation_force(hdb, flap2);

    // Simulation

    double dt = 0.005;

    system.SetTimeStep(dt);
    system.Initialize();

    flap1->Rotate(FrRotation(Direction(0., 1., 0.), -30.*DEG2RAD, NWU));

    bool use_irrlicht = true;

    if (use_irrlicht) {

        system.RunInViewer(0., 5, false);

    } else {

        double time = 0.;
        while (time < 20.) {

            time += dt;

            system.AdvanceTo(time);

            std::cout << "Time : " << time << " s" << std::endl;
        }
    }

    std::cout << " ================================================= End ================================ " << std::endl;

    return 0;

}