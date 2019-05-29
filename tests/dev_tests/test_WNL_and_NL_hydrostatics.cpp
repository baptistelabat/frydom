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
    FrOffshoreSystem system;
    system.SetName("Hydrostatics_platform");

    // --------------------------------------------------
    // Environment
    // --------------------------------------------------

    // ----- Ocean
    auto Ocean = system.GetEnvironment()->GetOcean();

    // ----- Seabed
    // Set the size of the seabed grid asset.
//    auto Seabed = Ocean->GetSeabed();
//    Seabed->GetSeabedGridAsset()->SetGrid(-300., 300., 100., -300., 300., 100.);

    // Set the bathymetry.
//    Seabed->SetBathymetry(-100, NWU);

    // Ramp.
//    system.GetEnvironment()->GetTimeRamp()->SetActive(true);
//    system.GetEnvironment()->GetTimeRamp()->SetByTwoPoints(0,0,20,1);

    // ----- Current
    // A uniform field is also set by default for the current model. In order to set the current characteristics,
    // you need to get first this uniform field.
//    auto current = Ocean->GetCurrent()->GetFieldUniform();
//    current->SetEast(6., KNOT, GOTO);

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
//    auto waveField = FreeSurface->SetAiryRegularOptimWaveField();
    auto waveField = FreeSurface->SetAiryRegularWaveField();

    // The Airy regular wave parameters are its height, period and direction.
    double waveHeight = 2.;
//    double waveHeight = 0.;
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
//    double Mass              = 3.22114e7;
    double Mass = 68321.544*1025; // Maillage visu.
//    double Mass = 53412.462*1025; // Maillage Nemoh.
//    Position platformCoG(0.22, 0.22, 2.92);
//    Position platformCoG(0., 0., 2.92);
    Position platformCoG(0.22, 0.22, 0.);
//    Position platformCoG(0., 0., 0.);
    FrFrame platformCoGFrame(platformCoG, FrRotation(), NWU);

    // Dof.
//    platform->GetDOFMask()->SetLock_X(true);
//    platform->GetDOFMask()->SetLock_Y(true);
//    platform->GetDOFMask()->SetLock_Z(true);
//    platform->GetDOFMask()->SetLock_Rx(true);
//    platform->GetDOFMask()->SetLock_Ry(true);
//    platform->GetDOFMask()->SetLock_Rz(true);

    // Inertia
    double Ixx               = 2.4e11;
    double Iyy               = 2.3e11;
    double Izz               = 2e12;
    FrInertiaTensor platformInertia(Mass, Ixx, Iyy, Izz, 0., 0., 0.,platformCoGFrame, NWU);

    platform->SetInertiaTensor(platformInertia);

    // Node.
    auto Node = platform->NewNode();
    Node->ShowAsset(true);
    Node->SetLogged(true);
    auto AssetNode = Node->GetAsset();
    AssetNode->SetSize(20);

    // Extra linear damping force.
    auto LinearDampingForce = make_linear_damping_force(platform, WATER, false);
    LinearDampingForce->SetDiagonalDamping(10e8,10e8,10e8,10e10,10e10,10e10);

    // -- Hydrodynamics

//     Create a hydrodynamic database (hdb), load data from the input file and creates and initialize the BEMBody.
//    auto hdb = make_hydrodynamic_database("Platform_HDB.hdb5");
    auto hdb = make_hydrodynamic_database("Platform_HDB_Without_drift.hdb5");

    // Create an equilibrium frame for the platform and add it to the system at the position of the body CoG.
    auto eqFrame = std::make_shared<FrEquilibriumFrame>(platform.get());
    system.AddPhysicsItem(eqFrame);

    // Map the equilibrium frame and the body in the hdb mapper
    hdb->Map(0, platform.get(), eqFrame);

    // -- Hydrodynamic mesh
    auto PlatformMesh = make_hydro_mesh(platform, "mesh_Platform_GVA7500_Sym.obj", FrFrame(), FrHydroMesh::ClippingSupport::WAVESURFACE);

    // -- Hydrostatics
    // Create the linear hydrostatic force and add it to the platform

    // Linear hydrostatic loads with a hydrostatic stiffness matrix from the HDB5 file.
//    auto forceHst = make_linear_hydrostatic_force(hdb, platform);

    // Linear hydrostatic loads with a hydrostatic stiffness matrix given in input of frydom.
//    auto forceHst = make_linear_hydrostatic_force(eqFrame, platform);
//    FrLinearHydrostaticStiffnessMatrix HydrostaMat;
//    HydrostaMat.SetK33(100);
//    forceHst->SetStiffnessMatrix(HydrostaMat);

    // Linear hydrostatic loads with a hydrostatic stiffness matrix computed with FrMesh.
//    auto forceHst = make_linear_hydrostatic_force(hdb, platform,"Platform_GVA7500.obj");

    // Nonlinear hydrostatic loads with the input mesh used for the visualization.
//    auto forceHst = make_nonlinear_hydrostatic_force(&system,platform,"Platform_GVA7500.obj");
//    auto forceHst = make_nonlinear_hydrostatic_force(&system,platform,"mesh_Platform_GVA7500_Sym.obj");

    auto forceHst = make_nonlinear_hydrostatic_force(platform,PlatformMesh);

    forceHst->ShowAsset(true);
    auto ForceHstAsset = forceHst->GetAsset();
    ForceHstAsset->SetSize(0.00000015);

    // -- Excitation
    // Create the linear excitation force and add it to the platform
//    auto excitationForce = make_linear_excitation_force(hdb, platform);
//    excitationForce->SetLogged(true);

    // Create the nonlinear excitation force and add it to the platform
    auto NonlinFKForce = make_nonlinear_froude_krylov_force(platform,PlatformMesh);

    // -- Radiation
//    auto radiationModel = make_radiation_convolution_model(hdb, &system);
//    radiationModel->SetImpulseResponseSize(platform.get(), 40., 0.01);

    // -- Wave drift force
    //TODO
//    auto waveDriftForce = std::make_shared<>(hdb.get());
//    platform->AddExternalForce(waveDriftForce);

    // -- Current model force, based on polar coefficients
    // Create the current model force and add it to the platform
//    auto currentForce = make_current_force("Platform_PolarCurrentCoeffs_NC.json", platform);

    // -- Wind model force, based on polar coefficients
    // Create the model model force and add it to the platform
//    auto windForce = make_wind_force("Platform_PolarWindCoeffs_NC.json", platform);
//    windForce->SetIsForceAsset(true);

    // ------------------ Run with Irrlicht ------------------ //

    // You can change the dynamical simulation time step using.
    system.SetTimeStep(0.01);
//    system.SetTimeStep(0.1);

    // Now you are ready to perform the simulation and you can watch its progression in the viewer. You can adjust
    // the time length of the simulation (here 60) and the distance from the camera to the objectif (300m).
    // For saving snapshots of the simulation, just turn the boolean to true.
//    system.Visualize(50.,false);
    system.RunInViewer(1000, 200, false);

}
