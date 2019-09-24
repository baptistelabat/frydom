//
// Created by camille on 09/05/19.
//

#include "frydom/frydom.h"

using namespace frydom;

// -----------------------------------------------------------------------
// FOSWEC Model definition
// -----------------------------------------------------------------------

void DemoModel(FrOffshoreSystem& system, bool flap1_fixed, bool flap2_fixed, double initial_angle) {

    // System

    system.GetPathManager()->SetLogFrameConvention(NWU);
    system.GetPathManager()->SetLogOutputPath("../results/");
    system.GetPathManager()->SetResourcesPath(std::string(RESOURCES_PATH));
    system.GetEnvironment()->GetOcean()->GetFreeSurface()->GetFreeSurfaceGridAsset()->SetGrid(-1, 1, 2, -1, 1, 2);

    // PLatform

    auto platform = system.NewBody();
    platform->SetName("platform");
    platform->AddMeshAsset(system.GetDataPath("FullPlatform.obj"));

    FrInertiaTensor inertia_b(153.8, 37.88, 29.63, 1., 0., 0., 0., Position(0., 0., 0.460), NWU);
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
    flap1->AddMeshAsset(system.GetDataPath("FullFlap_mesh.obj"));
    flap1->SetPosition(Position(-0.65, 0., -0.29), NWU);

    FrInertiaTensor inertia_f1(23.1, 1.42, 1.19, 1.99, 0., 0., 0., Position(0., 0., 0.), NWU);
    flap1->SetInertiaTensor(inertia_f1);

    auto node_1f = flap1->NewNode();
    node_1f->SetPositionInBody(Position(0., 0., -0.2108), NWU);
    node_1f->RotateAroundXInBody(-90. * DEG2RAD, NWU);

    if (flap1_fixed) {
        auto rev1 = make_fixed_link(node_1f, node_1b, &system);
    } else {
        auto rev1 = make_revolute_link(node_1f, node_1b, &system);
    }

    if (std::abs(initial_angle) > DBL_EPSILON) {
        flap1->RotateAroundPointInBody(FrRotation(Direction(0., 1., 0.), initial_angle * DEG2RAD, NWU),
                                       node_1f->GetNodePositionInBody(NWU), NWU);
    }
    // Flap2

    auto flap2 = system.NewBody();
    flap2->SetName("flap2");
    flap2->AddMeshAsset(system.GetDataPath("FullFlap_mesh.obj"));
    flap2->SetPosition(Position(0.65, 0., -0.29), NWU);

    FrInertiaTensor inertia_f2(23.1, 1.42, 1.19, 1.99, 0., 0., 0., Position(0., 0., 0.), NWU);
    flap2->SetInertiaTensor(inertia_f2);

    auto node_2f = flap2->NewNode();
    node_2f->SetPositionInBody(Position(0., 0., -0.2108), NWU);
    node_2f->RotateAroundXInBody(-90. * DEG2RAD, NWU);

    if (flap2_fixed) {
        auto rev2 = make_fixed_link(node_2f, node_2b, &system);
    } else {
        auto rev2 = make_revolute_link(node_2f, node_2b, &system);
    }

    // Hydrodynamic and radiation model

    auto hdb = make_hydrodynamic_database(system.GetDataPath("FOSWEC_phase2_filtered.hdb5"));

    auto eqFrame0 = std::make_shared<FrEquilibriumFrame>(platform.get());
    auto eqFrame1 = std::make_shared<FrEquilibriumFrame>(Position(-0.65, 0., -0.29), FrRotation(), NWU, flap1.get());
    auto eqFrame2 = std::make_shared<FrEquilibriumFrame>(Position(0.65, 0., -0.29), FrRotation(), NWU, flap2.get());

    system.Add(eqFrame0);
    system.Add(eqFrame1);
    system.Add(eqFrame2);

    hdb->Map(0, flap1.get(), eqFrame1);
    hdb->Map(1, flap2.get(), eqFrame2);
    hdb->Map(2, platform.get(), eqFrame0);

    auto radiationModel = make_radiation_convolution_model(hdb, &system);
    radiationModel->SetImpulseResponseSize(flap1.get(), 60., 0.01);
    radiationModel->SetImpulseResponseSize(flap2.get(), 60., 0.01);
    radiationModel->SetImpulseResponseSize(platform.get(), 60., 0.01);

    // Hydrostatic

    //--> Linear
    auto forceHst1 = make_linear_hydrostatic_force(hdb, flap1);
    auto forceHst2 = make_linear_hydrostatic_force(hdb, flap2);

    //--> Nonlinear
    //auto frameOffset1 = FrFrame(Position(0., 0., 0.), FrRotation(), NWU);
    //auto flapMesh1 = make_hydro_mesh(flap1, resources_path + "FullFlap1_fine.obj", frameOffset1,
    //                                 FrHydroMesh::ClippingSupport::PLANESURFACE);
    //auto forceHst1 = make_nonlinear_hydrostatic_force(flap1, flapMesh1);

    //auto frameOffset2 = FrFrame(Position(0., 0., 0.), FrRotation(), NWU);
    //auto flapMesh2 = make_hydro_mesh(flap2, resources_path + "FullFlap1_fine.obj", frameOffset2,
    //                                 FrHydroMesh::ClippingSupport::PLANESURFACE);
    //auto forceHst2 = make_nonlinear_hydrostatic_force(flap2, flapMesh2);

    // Excitation force

    //--> Linear
    auto excitation1 = make_linear_excitation_force(hdb, flap1);
    auto excitation2 = make_linear_excitation_force(hdb, flap2);

    // Viscous damping

    auto morisonModel_1 = make_morison_model(flap1.get());
    morisonModel_1->AddElement(Position(0., 0., -0.16), Position(0., 0., 0.42), 0.1, 0., 8., 0., 30);
    auto morisonForce_1 = make_morison_force(morisonModel_1, flap1);

    auto morisonModel_2 = make_morison_model(flap2.get());
    morisonModel_2->AddElement(Position(0., 0., -0.16), Position(0., 0., 0.42), 0.1, 0., 8., 0., 30);
    auto morisonForce_2 = make_morison_force(morisonModel_2, flap2);
}


// ------------------------------------------------------------------------
// Definition of the environment conditions
// ------------------------------------------------------------------------

static void SetEnvironment(FrOffshoreSystem& system, double period, double amplitude) {

    auto waveField = system.GetEnvironment()->GetOcean()->GetFreeSurface()->SetAiryRegularWaveField();
    waveField->SetWaveHeight(amplitude);
    waveField->SetWavePeriod(period);
    waveField->SetDirection(0., DEG, NWU, GOTO);

    system.GetEnvironment()->GetTimeRamp()->SetByTwoPoints(0., 0., 10., 1.);
    system.GetEnvironment()->GetTimeRamp()->SetActive(true);
}

// ------------------------------------------------------------------------------
// Simulation launcher
// ------------------------------------------------------------------------------

void Run(FrOffshoreSystem& system) {

    double dt = 0.01;

    system.SetTimeStep(dt);
    system.Initialize();

    bool use_irrlicht = true;

    if (use_irrlicht) {

        system.RunInViewer(0., 5, false);

    } else {

        double time = 0.;
        while (time < 35.) {

            time += dt;

            system.AdvanceTo(time);

            std::cout << "Time : " << time << " s" << std::endl;
        }
    }

}

// ----------------------------------------------------------------------
// Demo configuration 1 regular wave
// ----------------------------------------------------------------------

void DemoConfig1Reg() {

    // System
    FrOffshoreSystem system;
    system.SetName("FOSWEC_config1Reg");
    // Environment
    SetEnvironment(system, 3.333, 0.0225);
    // Configuration
    DemoModel(system, false, true, 0.);
    // Run
    Run(system);
}

// -----------------------------------------------------------------------
// Demo configuration 2 regular wave
// -----------------------------------------------------------------------

void DemoConfig2Reg() {

    // System
    FrOffshoreSystem system;
    system.SetName("FOSWEC_config2Reg");
    // Environment
    SetEnvironment(system, 3.333, 0.0225);
    // Configuration
    DemoModel(system, false, false, 0.);
    // Run
    Run(system);
}

// ------------------------------------------------------------------------
// Demo Wave excitation
// ------------------------------------------------------------------------

void DemoWaveExcitation() {

    // System
    FrOffshoreSystem system;
    system.SetName("FOSWEC_waveExcitation");
    // Environment
    SetEnvironment(system, 3.333, 0.0225);
    // Configuration
    DemoModel(system, false, false, 0.);
    // Run
    Run(system);
}

// --------------------------------------------------------------------------
// Demo Flap decay test
// --------------------------------------------------------------------------

void DemoFlapDecay() {

    // System
    FrOffshoreSystem system;
    system.SetName("FOSWEC_flapDecay");
    // Environment
    SetEnvironment(system, 10., 0.);
    // Configuration
    DemoModel(system, false, false, 10.);
    // Run
    Run(system);
}

int main(int argc, char* argv[]) {

    std::cout << " ============================================== Demo FOSWEC ============================= " << std::endl;

    std::cout << "Flap Decay..." << std::endl;
    DemoFlapDecay();

    //std::cout << "Wave exciation..." << std::endl;
    //DemoWaveExcitation();

    //std::cout << "Configuration 1 regular wave..." << std::endl;
    //DemoConfig1Reg();

    //std::cout << "Configuration 2 regular wave..." << std::endl;
    //DemoConfig2Reg();

    std::cout << " ================================================= End ================================ " << std::endl;

    return 0;
}