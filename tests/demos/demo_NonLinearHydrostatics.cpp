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

    /** This demo presents two non linear hydrostatic models. In both, the hydrostatic force is computed by integrating
     * the hydrostatic pressure over the wetted surface. In the weakly non linear hydrostatic model, the wetted surface
     * is defined by the exact position of the body, clipped by the mean free surface plane (z = 0). In the non linear
     * hydrostatic model, the wetted surface is still defined by the exact position of the body, but delimited by the
     * incident wave field.
     *
     * This demo considers a cylinder with a mesh of 2900 elements, over which both models can be applied. (select the one
     * you want to test).
     */

    // Define the frame convention (NWU for North-West-Up or NED for North-East-Down)
    FRAME_CONVENTION fc = NWU;

    // Define the wave direction convention (GOTO or COMEFROM), can be used also for current and wind direction definition.
    DIRECTION_CONVENTION dc = GOTO;

    // Create an offshore system, it contains all physical objects : bodies, links, but also environment components
    FrOffshoreSystem system;
    system.SetName("NonLinearHydrostatics");

    system.GetEnvironment()->GetOcean()->SetDensity(1023.);

    // Resources path
    cppfs::FilePath resources_path(std::string(RESOURCES_PATH));

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

    // ----- Free surface
    auto FreeSurface = Ocean->GetFreeSurface();
    // To manipulate the free surface grid asset, you first need to access it, through the free surface object.
    auto FSAsset = FreeSurface->GetFreeSurfaceGridAsset();

    // Set the size of the free surface grid asset.
    FSAsset->SetGrid(-3., 3, 1, -3, 3, 1);

    // --------------------------------------------------
    // Cylinder
    // --------------------------------------------------
    // Create the body, name it and make it cylinder
    auto cylinder = system.NewBody();
    cylinder->SetName("Cylinder");
    cylinder->SetColor(Yellow);

    double mass = MU_PI*0.2*0.2*0.1*system.GetEnvironment()->GetFluidDensity(WATER);
    makeItCylinder(cylinder, 0.2, 0.2, mass);

    // makeItCylinder create a horizontal cylinder. Rotate it 90° to have it vertical
    FrRotation cylRotation;    cylRotation.RotX_DEGREES(90., NWU);
    cylinder->Rotate(cylRotation);

    FrRotation decayRot; decayRot.RotX_DEGREES(10., NWU);
    // For decay test, offset its vertical position
    cylinder->SetPosition(Position(0.,0.,0.02), NWU);
    cylinder->Rotate(decayRot);

    // Extra linear damping force.
    auto LinearDampingForce = make_linear_damping_force(cylinder, WATER, false);
    LinearDampingForce->SetDiagonalDamping(10e0,10e0,10e0,10e0,10e0,10e0);

    // There is an offset in rotation between the initial orientations of the body and the mesh
    FrFrame meshOffset;
    meshOffset.SetRotation(cylRotation);

    // -- Hydrodynamic mesh
    auto CylinderMesh = make_hydro_mesh(cylinder, resources_path.resolve("Free_cylinder_2900_panels.obj").path(),
            meshOffset, FrHydroMesh::ClippingSupport::PLANESURFACE);

    // Writing the initial mesh, for checking only
    CylinderMesh->GetInitialMesh().Write("Mesh_Initial.obj");

//    // -- Hydrostatic stiffness matrix, for checking only
//    auto eqFrame = std::make_shared<FrEquilibriumFrame>(cylinder.get());
//    auto linearHSForce = make_linear_hydrostatic_force(eqFrame, cylinder, "Free_cylinder_2900_panels.obj", FrFrame());

    // -- Hydrostatics NL
    auto forceHst = make_nonlinear_hydrostatic_force(cylinder,CylinderMesh);


    // ------------------ Run with Irrlicht ------------------ //

    // You can change the dynamical simulation time step using.
    system.SetTimeStep(0.01);

//    // You can solve the static equilibrium first and visualize it.
//    system.GetStaticAnalysis()->SetNbSteps(10);
//    system.SolveStaticWithRelaxation();
//    system.Visualize(20,false);

    // Now you are ready to perform the simulation and you can watch its progression in the viewer. You can adjust
    // the time length of the simulation (here infinite) and the distance from the camera to the objectif (2m).
    // For saving snapshots of the simulation, just turn the boolean to true.
    system.RunInViewer(0., 2, false);

}
