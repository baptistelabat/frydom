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
    system.SetName("Hydrostatics_Cylinder");

    // --------------------------------------------------
    // Environment
    // --------------------------------------------------

    // ----- Ocean
    auto Ocean = system.GetEnvironment()->GetOcean();

    // ----- Seabed
    // Set the size of the seabed grid asset.
    auto Seabed = Ocean->GetSeabed();
    Seabed->GetSeabedGridAsset()->SetGrid(-3., 3., 1., -3., 3., 1.);

    // Set the bathymetry.
    Seabed->SetBathymetry(-5, NWU);

    // Ramp.
//    system.GetEnvironment()->GetTimeRamp()->SetActive(true);
//    system.GetEnvironment()->GetTimeRamp()->SetByTwoPoints(0,0,20,1);

    // ----- Free surface
    auto FreeSurface = Ocean->GetFreeSurface();
    // To manipulate the free surface grid asset, you first need to access it, through the free surface object.
    auto FSAsset = FreeSurface->GetFreeSurfaceGridAsset();

    // Set the size of the free surface grid asset.
    FSAsset->SetGrid(-3., 3, 1, -3, 3, 1);

    // You have to specify if you want the free surface asset to be updated during the simulation. By default, the
    // update is not activated.
    FSAsset->SetUpdateStep(10);

    // ----- WaveField
//    auto waveField = FreeSurface->SetAiryRegularOptimWaveField();
    auto waveField = FreeSurface->SetAiryRegularWaveField();

    // The Airy regular wave parameters are its height, period and direction.
    double waveHeight = 0.;
//    double waveHeight = 0.005;
    double wavePeriod = 2*3.14159/8;
    Direction waveDirection = Direction(SOUTH(fc));
//    Direction waveDirection = Direction(NORTH(fc));

    waveField->SetWaveHeight(waveHeight);
    waveField->SetWavePeriod(wavePeriod);
    waveField->SetDirection(waveDirection, fc, dc);
    
    // --------------------------------------------------
    // platform
    // --------------------------------------------------

    auto cylinder = system.NewBody();
    cylinder->SetName("Cylinder");
    cylinder->AddMeshAsset("Free_cylinder_Coarse.obj");
    cylinder->SetColor(Yellow);

    // Inertia Tensor
    double Mass = 12.88; // Maillage visu.
    Position cylinderCoG(0., 0., 0.);
    FrFrame cylinderCoGFrame(cylinderCoG, FrRotation(), NWU);

    // Dof.
//    cylinder->GetDOFMask()->SetLock_X(true);
//    cylinder->GetDOFMask()->SetLock_Y(true);
//    cylinder->GetDOFMask()->SetLock_Z(true);
//    cylinder->GetDOFMask()->SetLock_Rx(true);
//    cylinder->GetDOFMask()->SetLock_Ry(true);
//    cylinder->GetDOFMask()->SetLock_Rz(true);

    // Inertia
    double Ixx               = 0.5;
    double Iyy               = 0.5;
    double Izz               = 0.5;
    FrInertiaTensor cylinderInertia(Mass, Ixx, Iyy, Izz, 0., 0., 0.,cylinderCoGFrame, NWU);

    cylinder->SetInertiaTensor(cylinderInertia);

    // Node.
    auto Node = cylinder->NewNode();
    Node->ShowAsset(true);
    Node->SetLogged(true);
    auto AssetNode = Node->GetAsset();
    AssetNode->SetSize(20);

    // Extra linear damping force.
//    auto LinearDampingForce = make_linear_damping_force(cylinder, WATER, false);
//    LinearDampingForce->SetDiagonalDamping(10e1,10e1,10e1,10e1,10e0,10e1);

    // -- Hydrodynamics

//     Create a hydrodynamic database (hdb), load data from the input file and creates and initialize the BEMBody.
//    auto hdb = make_hydrodynamic_database("Platform_HDB.hdb5");
    auto hdb = make_hydrodynamic_database("Platform_HDB_Without_drift.hdb5");

    // Create an equilibrium frame for the platform and add it to the system at the position of the body CoG.
    auto eqFrame = std::make_shared<FrEquilibriumFrame>(cylinder.get());
    system.AddPhysicsItem(eqFrame);

    // Map the equilibrium frame and the body in the hdb mapper
    hdb->Map(0, cylinder.get(), eqFrame);

    // -- Hydrostatics
    // Create the linear hydrostatic force and add it to the platform

    // Linear hydrostatic loads with a hydrostatic stiffness matrix from the HDB5 file.
//    auto forceHst = make_linear_hydrostatic_force(hdb, cylinder);

    // Linear hydrostatic loads with a hydrostatic stiffness matrix given in input of frydom.
//    auto forceHst = make_linear_hydrostatic_force(hdb, cylinder);
//    FrLinearHydrostaticStiffnessMatrix HydrostaMat;
//    HydrostaMat.SetK33(100);
//    forceHst->SetStiffnessMatrix(HydrostaMat);

    // Linear hydrostatic loads with a hydrostatic stiffness matrix computed with FrMesh.
//    auto forceHst = make_linear_hydrostatic_force(hdb, cylinder,"Platform_GVA7500.obj");

    // Weakly nonlinear hydrostatic loads with the input mesh used for the visualization.
//    auto forceHst = make_weakly_nonlinear_hydrostatic_force(&system,hdb,cylinder,"Platform_GVA7500.obj"); // Visu.

    // Weakly nonlinear hydrostatic loads with the input mesh used for the Nemoh computation.
//    auto forceHst = make_weakly_nonlinear_hydrostatic_force(&system,hdb,cylinder,"mesh_Platform_GVA7500.obj"); // Nemoh.

    // Weakly nonlinear hydrostatic loads with the input mesh used for the Nemoh computation with a symmetry plane.
//    auto forceHst = make_weakly_nonlinear_hydrostatic_force(&system,hdb,cylinder,"mesh_Platform_GVA7500_Sym.obj"); // Nemoh sym.

    // Nonlinear hydrostatic loads with the input mesh used for the visualization.
    auto forceHst = make_nonlinear_hydrostatic_force(&system,hdb,cylinder,"Free_cylinder_Coarse.obj");

    forceHst->SetLogged(true);
    forceHst->ShowAsset(true);
    auto ForceHstAsset = forceHst->GetAsset();
    ForceHstAsset->SetSize(0.00000015);

    // -- Excitation
    // Create the linear excitation force and add it to the platform
//    auto excitationForce = make_linear_excitation_force(hdb, platform);

    // -- Radiation
//    auto radiationModel = make_radiation_convolution_model(hdb, &system);
//    radiationModel->SetImpulseResponseSize(platform.get(), 40., 0.01);


    // ------------------ Run with Irrlicht ------------------ //

    // You can change the dynamical simulation time step using.
    system.SetTimeStep(0.01);
//    system.SetTimeStep(0.1);

    // Now you are ready to perform the simulation and you can watch its progression in the viewer. You can adjust
    // the time length of the simulation (here 60) and the distance from the camera to the objectif (300m).
    // For saving snapshots of the simulation, just turn the boolean to true.
    system.RunInViewer(100, 2, false);

}
