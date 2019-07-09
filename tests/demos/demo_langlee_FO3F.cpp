//
// Created by camille on 12/04/19.
//

#include "frydom/frydom.h"

using namespace frydom;

// --------------------------------------------------------------------
// Buoyancy force
// --------------------------------------------------------------------

class FrDiffBuoyancyForce : public FrForce {

public:

    void Compute(double time) override {

        double grav = GetSystem()->GetEnvironment()->GetGravityAcceleration();
        double massPTO = 1.e5;
        double mg = massPTO * grav;

        SetForceInWorldAtCOG(Force(0., 0., mg), NWU);

    }

    void StepFinalize() override { }

};

// --------------------------------------------------------------------
// Main
// --------------------------------------------------------------------

int main(int argc, char* argv[]) {

    std::cout << " ====================================== Demo Langlee F3OF ========================== " << std::endl;

    // System

    FrOffshoreSystem system;

    //system.SetSolver(FrOffshoreSystem::SOLVER::MINRES);
    //system.SetSolverVerbose(true);

    system.GetPathManager()->SetLogOutputPath("../results/");
    system.GetPathManager()->SetLogFrameConvention(NWU);
    system.SetName("Langlee");

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
    barge->AddMeshAsset("barge_draft8_5.obj");
    barge->SetPosition(Position(0., 0., -8.5), NWU);

    double mass_b = 1.089825e6;
    double Iy_b = 7.63e7;
    FrInertiaTensor InertiaTensor(mass_b, Iy_b, Iy_b, Iy_b, 0., 0., 0., Position(0., 0., 0.), NWU);
    barge->SetInertiaTensor(InertiaTensor);

    barge->GetDOFMask()->MakeItLocked();

    auto node_1b = barge->NewNode();
    node_1b->SetPositionInBody(Position(-12.5, 0., 0.), NWU);
    node_1b->RotateAroundXInBody(-90.*DEG2RAD, NWU);

    auto node_2b = barge->NewNode();
    node_2b->SetPositionInBody(Position(12.5, 0., 0.), NWU);
    node_2b->RotateAroundXInBody(-90.*DEG2RAD, NWU);

    // Flap1

    // -- Position
    auto flap1 = system.NewBody();
    flap1->SetName("flap1");
    flap1->AddMeshAsset("FullFlap1.obj");
    flap1->SetPosition(Position(-12.5, 0., -8.5), NWU);

    // -- Inertia
    double mass_f1 = 1.7925e5; //;2.8874e5
    double Iy_f1 = 1.3e6; //1.3e6;
    FrFrame COGFrame_flap(Position(0., 0., 3.5), FrRotation(), NWU);
    FrInertiaTensor InertiaTensor_flap(mass_f1, Iy_f1, Iy_f1, Iy_f1, 0., 0., 0., Position(0., 0., 3.5), NWU);

    flap1->SetInertiaTensor(InertiaTensor_flap);

    // -- Link
    auto node_1f = flap1->NewNode();
    node_1f->SetPositionInBody(Position(0., 0., 0), NWU);
    node_1f->RotateAroundXInBody(-90.*DEG2RAD, NWU);

    auto rev1 = make_revolute_link(node_1f, node_1b, &system);

    // -- PTO
    //auto forcePTO_1 = make_linear_damping_force(flap1, WATER, false);
    //forcePTO_1->SetDiagonalRotationDamping(0., 1.e6, 0.);

    // -- Viscous damping
    //auto morisonModel_1 = make_morison_model(flap1.get());
    //morisonModel_1->AddElement(Position(0., 0., 0.), Position(0., 0., 11.), 2., 0., 8., 0., 10);
    //auto morisonForce_1 = make_morison_force(morisonModel_1, flap1);

    // Flap2

    // -- Position
    auto flap2 = system.NewBody();
    flap2->SetName("flap2");
    flap2->AddMeshAsset("FullFlap1.obj");
    flap2->SetPosition(Position(12.5, 0., -8.5), NWU);

    flap2->SetInertiaTensor(InertiaTensor_flap);

    // -- Link
    auto node_2f = flap2->NewNode();
    node_2f->SetPositionInBody(Position(0., 0., 0), NWU);
    node_2f->RotateAroundXInBody(-90.*DEG2RAD, NWU);

    auto rev2 = make_revolute_link(node_2f, node_2b, &system);

    // -- PTO
    //auto forcePTO_2 = make_linear_damping_force(flap2, WATER, false);
    //forcePTO_2->SetDiagonalRotationDamping(0., 1.e6, 0.);

    // -- Viscous damping
    //auto morisonModel_2 = make_morison_model(flap2.get());
    //morisonModel_2->AddElement(Position(0., 0., 0.), Position(0., 0., 11.), 2., 0., 8., 10);
    //auto morisonForce_2 = make_morison_force(morisonModel_2, flap2);

    // Hydrodynamic

    auto hdb = make_hydrodynamic_database("Langlee_draft8_5_filtered.hdb5");

    auto eqFrame0 = std::make_shared<FrEquilibriumFrame>(barge.get());
    auto eqFrame1 = std::make_shared<FrEquilibriumFrame>(flap1.get());
    auto eqFrame2 = std::make_shared<FrEquilibriumFrame>(flap2.get());

    hdb->Map(0, flap1.get(), eqFrame1);
    hdb->Map(1, flap2.get(), eqFrame2);
    hdb->Map(2, barge.get(), eqFrame0);

    auto radiationModel = make_radiation_convolution_model(hdb, &system);
    radiationModel->SetImpulseResponseSize(flap1.get(), 80., 0.01);
    radiationModel->SetImpulseResponseSize(flap2.get(), 80., 0.01);
    radiationModel->SetImpulseResponseSize(barge.get(), 80., 0.01);

    // Hydrostatic

    // -- Linear
    //auto forceHst1 = make_linear_hydrostatic_force(hdb, flap1);
    //auto forceHst2 = make_linear_hydrostatic_force(hdb, flap2);

    //auto diffBuoyForce1 = std::make_shared<FrDiffBuoyancyForce>();
    //flap1->AddExternalForce(diffBuoyForce1);

    //auto diffBuoyForce2 = std::make_shared<FrDiffBuoyancyForce>();
    //flap2->AddExternalForce(diffBuoyForce2);

    // -- Nonlinear
    auto flap1Mesh = make_hydro_mesh(flap1, "FullFlap_sym_wsep_draft8_5_fillet.obj", FrFrame(), FrHydroMesh::ClippingSupport::PLANESURFACE);
    auto forceHst1 = make_nonlinear_hydrostatic_force(flap1, flap1Mesh);

    auto flap2Mesh = make_hydro_mesh(flap2, "FullFlap_sym_wsep_draft8_5_fillet.obj", FrFrame(), FrHydroMesh::ClippingSupport::PLANESURFACE);
    auto forceHst2 = make_nonlinear_hydrostatic_force(flap2, flap2Mesh);

    //flap1Mesh->GetInitialMesh().Write("HydroMesh_Flap1_Initial.obj");
    //flap2Mesh->GetInitialMesh().Write("HydroMesh_Flap2_Initial.obj");

    // Excitation

    //auto excitation1 = make_linear_excitation_force(hdb, flap1);
    //auto excitation2 = make_linear_excitation_force(hdb, flap2);

    // Simulation

    auto dt = 0.005;

    system.SetTimeStep(dt);

    system.Initialize();

    flap1->Rotate(FrRotation(Direction(0,1,0), 10.*DEG2RAD, NWU));

    bool is_irrlicht = true;

    if (is_irrlicht) {
        system.RunInViewer(0, 50, false);
    } else {
        auto time = 0.;
        while (time < 300.) {
            time += dt;
            system.AdvanceTo(time);
            std::cout << "Time : " << time << " s" << std::endl;
        }
    }

    std::cout << " ==================================== End ========================== " << std::endl;

    return 0;

}

