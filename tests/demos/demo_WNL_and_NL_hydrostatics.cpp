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

using namespace frydom;

int main(int argc, char* argv[]) {

    // Define the frame convention (NWU for North-West-Up or NED for North-East-Down)
    FRAME_CONVENTION fc = NWU;

    // Define the wave direction convention (GOTO or COMEFROM), can be used also for current and wind direction definition.
    DIRECTION_CONVENTION dc = GOTO;

    // Create an offshore system, it contains all physical objects : bodies, links, but also environment components
    FrOffshoreSystem_ system;

    // --------------------------------------------------
    // Environment
    // --------------------------------------------------

    // ----- Ocean
    auto Ocean = system.GetEnvironment()->GetOcean();

    // ----- Seabed
    // Set the size of the seabed grid asset.
    auto Seabed = Ocean->GetSeabed();
    Seabed->GetSeabedGridAsset()->SetGrid(-300., 300., 100., -300., 300., 100.);

    // Set the bathymetry.
    Seabed->SetBathymetry(-100, NWU);

    // Ramp.
    system.GetEnvironment()->GetTimeRamp()->SetActive(true);
    system.GetEnvironment()->GetTimeRamp()->SetByTwoPoints(0,0,20,1);

    // ----- Current
    // A uniform field is also set by default for the current model. In order to set the current characteristics,
    // you need to get first this uniform field.
    auto current = Ocean->GetCurrent()->GetFieldUniform();
    current->SetEast(6., KNOT, GOTO);

    // ----- Wind
    // The wind model is set exactly in the same manner as the current:
//    auto wind = system.GetEnvironment()->GetAtmosphere()->GetWind()->GetFieldUniform();
//    wind->SetNorth(6., KNOT, GOTO);

    // ----- Free surface
    auto FreeSurface = Ocean->GetFreeSurface();
    // To manipulate the free surface grid asset, you first need to access it, through the free surface object.
    auto FSAsset = FreeSurface->GetFreeSurfaceGridAsset();

    // Set the size of the free surface grid asset.
    FSAsset->SetGrid(-200., 200, 10, -200, 200, 10);

    // You have to specify if you want the free surface asset to be updated during the simulation. By default, the
    // update is not activated.
    FSAsset->SetUpdateStep(10);

    // ----- WaveField
    auto waveField = FreeSurface->SetAiryRegularOptimWaveField();

    // The Airy regular wave parameters are its height, period and direction.
//    double waveHeight = 2.;
    double waveHeight = 0.;
    double wavePeriod = 10.;
    Direction waveDirection = Direction(SOUTH(fc));

    waveField->SetWaveHeight(waveHeight);
    waveField->SetWavePeriod(wavePeriod);
    waveField->SetDirection(waveDirection, fc, dc);
    
    // --------------------------------------------------
    // platform
    // --------------------------------------------------

    auto platform = system.NewBody();
    platform->SetName("platform");
    platform->AddMeshAsset("Platform_GVA7500.obj");
    platform->SetColor(Yellow);

    // Inertia Tensor
    double Mass              = 3.22114e7;
    Position platformCoG(0.22, 0.22, 2.92);
    FrFrame_ platformCoGFrame(platformCoG, FrRotation_(), NWU);

    // Inertia
    double Ixx               = 2.4e11;
    double Iyy               = 2.3e11;
    double Izz               = 2e12;
    FrInertiaTensor_ platformInertia(Mass, Ixx, Iyy, Izz, 0., 0., 0.,platformCoGFrame, NWU);

    platform->SetInertiaTensor(platformInertia);

    // -- Hydrodynamics

//     Create a hydrodynamic database (hdb), load data from the input file and creates and initialize the BEMBody.
//    auto hdb = make_hydrodynamic_database("Platform_HDB.hdb5");
    auto hdb = make_hydrodynamic_database("Platform_HDB_Without_drift.hdb5");

    // Create an equilibrium frame for the platform and add it to the system at the position of the body CoG.
    auto eqFrame = std::make_shared<FrEquilibriumFrame_>(platform.get());
    system.AddPhysicsItem(eqFrame);

    // Map the equilibrium frame and the body in the hdb mapper
    hdb->Map(0, platform.get(), eqFrame);

    // -- Hydrostatics
    // Create the linear hydrostatic force and add it to the platform
//    auto forceHst = make_linear_hydrostatic_force(hdb, platform);
    auto forceHst = make_weakly_nonlinear_hydrostatic_force(hdb,platform,"Platform_GVA7500.obj");

    // -- Excitation
    // Create the linear excitation force and add it to the platform
//    auto excitationForce = make_linear_excitation_force(hdb, platform);

    // -- Radiation
//    auto radiationModel = make_radiation_convolution_model(hdb, &system);
//    radiationModel->SetImpulseResponseSize(platform.get(), 40., 0.01);

    // -- Wave drift force
    //TODO
//    auto waveDriftForce = std::make_shared<>(hdb.get());
//    platform->AddExternalForce(waveDriftForce);

    // -- Current model force, based on polar coefficients
    // Create the current model force and add it to the platform
//    auto currentForce = make_current_force("Platform_PolarCurrentCoeffs_NC.yml", platform);

    // -- Wind model force, based on polar coefficients
    // Create the model model force and add it to the platform
//    auto windForce = make_wind_force("Platform_PolarWindCoeffs_NC.yml", platform);
//    windForce->SetIsForceAsset(true);

    // ------------------ Run with Irrlicht ------------------ //

    // You can change the dynamical simulation time step using.
    system.SetTimeStep(0.01);

    // Now you are ready to perform the simulation and you can watch its progression in the viewer. You can adjust
    // the time length of the simulation (here 60) and the distance from the camera to the objectif (300m).
    // For saving snapshots of the simulation, just turn the boolean to true.
//    system.Visualize(50.,false);
    system.RunInViewer(100, 200, false);

}
