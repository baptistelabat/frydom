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

class AddedMassRadiationForce {

protected :
    FrHydroDB* m_HDB;
    GeneralizedForce m_force;
    FrBody* m_body;

public:

    AddedMassRadiationForce(FrHydroDB* HDB, FrBody* body) : m_HDB(HDB), m_body(body) {}

    void Update(FrBody* body) {

        auto BEMBody = m_HDB->GetBody(body);
        auto infiniteAddedMass = BEMBody->GetInfiniteAddedMass(BEMBody);

        GeneralizedAcceleration acc;
        acc.SetAcceleration(body->GetCOGLinearAccelerationInWorld(NWU));
        acc.SetAngularAcceleration(body->GetAngularAccelerationInBody(NWU));

        m_force = -infiniteAddedMass * acc;
    }

    Force GetForceInWorld() {
        return m_force.GetForce();
    }

    Torque GetTorqueInWorldAtCOG() {
        return m_force.GetTorque();
    }
};

int main(int argc, char* argv[]) {

    ////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
    //                                          Conventions
    ////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

    // Define the frame convention (NWU for North-West-Up or NED for North-East-Down)
    FRAME_CONVENTION fc = NWU;

    // Define the wave direction convention (GOTO or COMEFROM), can be used also for current and wind direction definition.
    DIRECTION_CONVENTION dc = GOTO;

    // Create an offshore system, it contains all physical objects : bodies, links, but also environment components
    FrOffshoreSystem system;
    system.SetName("Ellipsoid_OMAE2014");

    // Resources path
    cppfs::FilePath resources_path(std::string(RESOURCES_PATH));

    ////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
    //                                          Environment
    ////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

    // ----- Ocean
    auto Ocean = system.GetEnvironment()->GetOcean();
    Ocean->SetDensity(1025);

    // ----- Seabed
    // Set the size of the seabed grid asset.
    auto Seabed = Ocean->GetSeabed();
    Seabed->GetSeabedGridAsset()->SetGrid(-100., 100., 1., -100., 100., 1.);

    // Set the bathymetry.
    Seabed->SetBathymetry(-100, NWU);

    // Ramp.
//    system.GetEnvironment()->GetTimeRamp()->SetActive(true);
//    system.GetEnvironment()->GetTimeRamp()->SetByTwoPoints(0,0,20,1);

    ////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
    //                                     Free surface visualization
    ////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

    auto FreeSurface = Ocean->GetFreeSurface();
    // To manipulate the free surface grid asset, you first need to access it, through the free surface object.
    auto FSAsset = FreeSurface->GetFreeSurfaceGridAsset();

    // Set the size of the free surface grid asset.
    FSAsset->SetGrid(-100., 100, 1, -100, 100, 1);

    // You have to specify if you want the free surface asset to be updated during the simulation. By default, the
    // update is not activated.
    FSAsset->SetUpdateStep(10);

    ////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
    //                                        Incident wave field
    ////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

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

    ////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
    //                                                  Body 1
    ////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

    auto Ellipsoid = system.NewBody();
    Ellipsoid->SetName("Ellipsoid");
    Ellipsoid->AddMeshAsset(resources_path.resolve("elipsoid_scaled_2880_faces.obj").path());
    Ellipsoid->SetColor(Yellow);

    // Inertia Tensor
    // double Mass = 273730; // OMAE_2014.
    double Mass = 268344.1458; // Volume*1025.
    Position EllipsoidCoG(0., 0., 2.);

    // Dof.
    Ellipsoid->GetDOFMask()->SetLock_X(true);
    Ellipsoid->GetDOFMask()->SetLock_Y(true);
//    Ellipsoid->GetDOFMask()->SetLock_Z(true);
    Ellipsoid->GetDOFMask()->SetLock_Rx(true);
    Ellipsoid->GetDOFMask()->SetLock_Ry(true);
    Ellipsoid->GetDOFMask()->SetLock_Rz(true);

    // Inertia
    double Ixx               = 0.5;
    double Iyy               = 0.5;
    double Izz               = 2;
    FrInertiaTensor EllipsoidInertia(Mass, Ixx, Iyy, Izz, 0., 0., 0.,EllipsoidCoG, NWU);
    Ellipsoid->SetInertiaTensor(EllipsoidInertia);

    // Node.
    auto Node = Ellipsoid->NewNode();
    Node->ShowAsset(true);
    Node->SetLogged(true);
    auto AssetNode = Node->GetAsset();
    AssetNode->SetSize(20);

    // Set of the body position.
    // Position InitPos(0.,0.,-0.00957); // 2880 faces.
//    Position InitPos(0.,0.,-0.0046); // 4200 faces.
//    Ellipsoid->SetPosition(InitPos,NWU);

    ////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
    //                                          Hydrodynamic database
    ////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

    // Create a hydrodynamic database (hdb), load data from the input file and creates and initialize the BEMBody.
//    auto hdb = make_hydrodynamic_database("Ellipsoid_4500_faces_WAMIT_mesh_sym_mean_position.hdb5");
    auto hdb = make_hydrodynamic_database(resources_path.resolve("Ellipsoid_2774_faces_with_sym.hdb5").path());

    // Create an equilibrium frame for the platform and add it to the system at the position of the body CoG.
    // auto eqFrame = std::make_shared<FrEquilibriumFrame>(Position(0.,0.,1.99043),FrRotation(),NWU,Ellipsoid.get()); // 2880 faces.
    auto eqFrame = std::make_shared<FrEquilibriumFrame>(Position(0.,0.,2),FrRotation(),NWU,Ellipsoid.get()); // 4200 faces.
    system.AddPhysicsItem(eqFrame);

    // Map the equilibrium frame and the body in the hdb mapper
    hdb->Map(0, Ellipsoid.get(), eqFrame);

    ////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
    //                                           Linear hydrostatics
    ////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

    // INFO: The hydrostatic force object is called before this update to compute the hydrostatic stiffness matrix at the equilibrium position.

    // Linear.
    auto forceHst = make_linear_hydrostatic_force(eqFrame, Ellipsoid,resources_path.resolve("elipsoid_scaled_mean_position.obj").path(),FrFrame());
    forceHst->ShowAsset(true);
    auto ForceHstAsset = forceHst->GetAsset();
    ForceHstAsset->SetSize(0.00000015);

    ////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
    //                              Hydrodynamics and weakly or nonlinear hydrostatics
    ////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
    // New position of the body.
//    Position InitPos(0.,0.,-0.00957); // 2880 faces
//    Position InitPos(0.,0.,-0.0046); // 4200 faces
//    InitPos.Set(0.,0.,0.49043); // InitPos + 0.5 m for a heave decay test, 2880 faces.

    Position InitPos(0.,0.,0.5); // InitPos + 0.5 m for a heave decay test, 4200 faces.
    Ellipsoid->SetPosition(InitPos,NWU);

    // -- Hydrodynamic mesh.
    // auto EllipsoidMesh = make_hydro_mesh_nonlinear(Ellipsoid,"Ellipsoid_2880_faces.obj");
//    auto EllipsoidMesh = make_hydro_mesh(Ellipsoid,"Ellipsoid_4200_faces.obj", FrFrame(), true);
//    auto EllipsoidMesh = make_hydro_mesh(Ellipsoid,"elipsoid_scaled.obj", FrFrame(),false);

    // Weakly or fully nonlinear hydrostatics.
//    auto forceHst = make_nonlinear_hydrostatic_force(Ellipsoid,EllipsoidMesh);
//    forceHst->SetLogged(true);
//    forceHst->ShowAsset(true);
//    auto ForceHstAsset = forceHst->GetAsset();
//    ForceHstAsset->SetSize(0.00000015);

    // -- Radiation.
    auto radiationModel = make_radiation_convolution_model(hdb, &system);
    radiationModel->SetImpulseResponseSize(Ellipsoid.get(), 40., 0.02);


    ////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
    //                                          Extra linear damping force
    ////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

//    auto LinearDampingForce = make_linear_damping_force(Ellipsoid, WATER, false);
//    LinearDampingForce->SetDiagonalDamping(10e0,10e0,10e4,10e0,10e0,10e0);

    ////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
    //                                                    Run
    ////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

    // You can change the dynamical simulation time step using.
    system.SetTimeStep(0.01);

//    system.Initialize();

//    Position InitPos(0.,0.,0.5); // InitPos + 0.5 m for a heave decay test, 4200 faces.
//    Ellipsoid->SetPosition(InitPos,NWU);

    // Now you are ready to perform the simulation and you can watch its progression in the viewer. You can adjust
    // the time length of the simulation (here 60) and the distance from the camera to the objectif (300m).
    // For saving snapshots of the simulation, just turn the boolean to true.

    clock_t begin = clock();

    system.RunInViewer(25, 50, false);

//    auto time = 0.;
//    auto dt = 0.01;
//    while (time < 40.) {
//
//        time += dt;
//        system.AdvanceTo(time);
//
//        std::cout << "time : " << time << " ; position of the body = "
//                  << Ellipsoid->GetPosition(NWU).GetX() << " ; "
//                  << Ellipsoid->GetPosition(NWU).GetY() << " ; "
//                  << Ellipsoid->GetPosition(NWU).GetZ()
//                  << std::endl;
//
//        radiationAddedMassForce->Update(Ellipsoid.get());
//
//    }

    clock_t end = clock();
    double elapsed_secs = double(end - begin) / CLOCKS_PER_SEC;
    std::cout << "Elapsed time : " << elapsed_secs << std::endl;

}
