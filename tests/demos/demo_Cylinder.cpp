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

#include <ctime>

using namespace frydom;

int main(int argc, char* argv[]) {

    // Define the frame convention (NWU for North-West-Up or NED for North-East-Down)
    FRAME_CONVENTION fc = NWU;

    // Define the wave direction convention (GOTO or COMEFROM), can be used also for current and wind direction definition.
    DIRECTION_CONVENTION dc = GOTO;

    // Create an offshore system, it contains all physical objects : bodies, links, but also environment components
    FrOffshoreSystem system;
    system.SetName("Cylinder_2900_panels");

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
    auto waveField = FreeSurface->SetAiryRegularOptimWaveField();
//    auto waveField = FreeSurface->SetAiryRegularWaveField();
//    auto waveField = FreeSurface->SetAiryIrregularOptimWaveField();

    // The Airy regular wave parameters are its height, period and direction.
    double waveHeight = 0.;
//    double waveHeight = 0.005;
    double wavePeriod = 2*3.14159/8;
    Direction waveDirection = Direction(SOUTH(fc));
//    Direction waveDirection = Direction(NORTH(fc));

    waveField->SetWaveHeight(waveHeight);
    waveField->SetWavePeriod(wavePeriod);
    waveField->SetDirection(waveDirection, fc, dc);

//    // The Airy irregular wave parameters are based on the wave spectrum chosen : Jonswap or Pierson-Moskowitz.
//    // Set the JONSWAP wave spectrum : Significant height (Hs) and Peak period (Tp). A default Gamma for the Jonswap
//    // spectrum is set to 3.3.
//    double Hs = 0.01;    double Tp = 9;
//    auto Jonswap = waveField->SetJonswapWaveSpectrum(Hs, Tp);
//
//    // Define the wave frequency discretization. It is based on a linear discretization within the extrema given and
//    // using the number of frequency specified. With more frequency, it will be more realist but will take longer to
//    // simulate.
//    double w1 = 0.5; double w2 = 2; unsigned int nbFreq = 20;
//    waveField->SetWaveFrequencies(w1,w2,nbFreq);
//
//    // For a uni-directional wave, you just need to set the mean wave direction. You can also choose to set a
//    // direction angle from North direction (see SetMeanWaveDirectionAngle()).
//    waveField->SetMeanWaveDirection(Direction(SOUTH(fc)), fc, dc);
//
//    // For a directional wave, you also have to specify the spreading factor and the refinement wanted on the
//    // direction discretization. With more directions, it will be more realist but will take longer to simulate.
//    // The direction discretization is based on a linear discretization. The extrema are computed automatically
//    // using the spreading factor and the mean direction.
//    double spreadingFactor = 10.;    unsigned int nbDir = 10;
//    waveField->SetDirectionalParameters(nbDir, spreadingFactor);

    // --------------------------------------------------
    // Cylinder
    // --------------------------------------------------

    auto cylinder = system.NewBody();
    cylinder->SetName("Cylinder");
    cylinder->AddMeshAsset("Free_cylinder_2900_panels.obj");
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
    double Izz               = 2;
    FrInertiaTensor cylinderInertia(Mass, Ixx, Iyy, Izz, 0., 0., 0.,cylinderCoGFrame, NWU);

    cylinder->SetInertiaTensor(cylinderInertia);

    cylinder->SetPosition(Position(0.,0.,0.02), NWU);

    // Node.
    auto Node = cylinder->NewNode();
    Node->ShowAsset(true);
    Node->SetLogged(true);
    auto AssetNode = Node->GetAsset();
    AssetNode->SetSize(20);

    // Extra linear damping force.
    auto LinearDampingForce = make_linear_damping_force(cylinder, WATER, false);
    LinearDampingForce->SetDiagonalDamping(10e0,10e0,10e0,10e0,10e0,10e0);

    // -- Hydrodynamic mesh
    auto CylinderMesh = make_hydro_mesh_nonlinear(&system,cylinder,"Free_cylinder_2900_panels.obj");
//    auto CylinderMesh = make_hydro_mesh_weakly_nonlinear(&system,cylinder,"Free_cylinder_11600_panels.obj");
    mathutils::Matrix33<double> Rotation;
    Rotation.SetIdentity();
    Position MeshOffset(0,0,0);
    CylinderMesh->SetMeshOffsetRotation(MeshOffset,Rotation);
    CylinderMesh->GetInitialMesh().Write("Mesh_Initial.obj");

    // -- Hydrostatics
    auto forceHst = make_nonlinear_hydrostatic_force(&system,cylinder,CylinderMesh);
    forceHst->SetLogged(true);
    forceHst->ShowAsset(true);
    auto ForceHstAsset = forceHst->GetAsset();
    ForceHstAsset->SetSize(0.00000015);

    // ------------------ Run with Irrlicht ------------------ //

    // You can change the dynamical simulation time step using.
    system.SetTimeStep(0.01);

    // Now you are ready to perform the simulation and you can watch its progression in the viewer. You can adjust
    // the time length of the simulation (here 60) and the distance from the camera to the objectif (300m).
    // For saving snapshots of the simulation, just turn the boolean to true.

    system.GetStaticAnalysis()->SetNbSteps(10);
    system.SolveStaticWithRelaxation();

//    system.Visualize(20,false);



//    clock_t begin = clock();
//
    system.RunInViewer(20, 2, false);
//
//    clock_t end = clock();
//    double elapsed_secs = double(end - begin) / CLOCKS_PER_SEC;
//    std::cout << elapsed_secs << std::endl;

}
