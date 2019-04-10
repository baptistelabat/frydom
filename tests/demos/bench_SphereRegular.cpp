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


void ValidationResults(const std::vector<double> vtime, const std::vector<double> heave,
                       const int iperiod, const int isteepness) {

    FrHDF5Reader db("bench_sphere_regular.h5");

    auto path = "T" + std::to_string(iperiod) + "/H" + std::to_string(isteepness);

    auto rao_bench = db.ReadDouble(path + "/rao");
    auto wave_height = db.ReadDouble(path + "/wave_height");
    auto period = db.ReadDouble(path + "/period");
    auto steepness = db.ReadDouble(path + "/steepness");

    int it = 0;
    while (vtime[it] < 100.) {
        it += 1;
    }

//    auto motion = -999.;
//
//    for (int i=it; i < vtime.size(); i++) {
//        motion = std::max(motion, heave[i]);
//    }

    auto motionMax = -999.;
    for (int i=it; i < vtime.size(); i++) {
        motionMax = std::max(motionMax, heave[i]);
    }

    auto motionMin = 999.;
    for (int i=it; i < vtime.size(); i++) {
        motionMin = std::min(motionMin, heave[i]);
    }

//    auto rao = motion / (0.5 * wave_height);
    auto rao = ((motionMax - motionMin)*0.5) / (0.5 * wave_height);
    auto err_rel = std::abs(rao - rao_bench) / rao_bench;

    // Print results
    std::cout << "---------------------------------------------------------" << std::endl;
    std::cout << " T = " << period << " , H = " << wave_height << ", Steepness = " << steepness << std::endl;
    std::cout << " RAO = " << rao << " Bench : " << rao_bench << " Error rel. : " << err_rel << std::endl;

    // Print to file
    std::ofstream myfile;
    myfile.open("sphere_regular.csv", std::ios::out | std::ios::app);

    myfile << period << ";" << wave_height << ";" << steepness << ";"
           << rao << ";" << ";" << rao_bench << ";" << err_rel << std::endl;

    myfile.close();

}

std::vector<double> ReadParam(const std::string dbfile, const int iperiod, const int isteepness) {

    auto path = "T" + std::to_string(iperiod) + "/H" + std::to_string(isteepness);

    std::vector<double> param(2);

    FrHDF5Reader db(dbfile);

    param[0] = db.ReadDouble(path + "/period");
    param[1] = db.ReadDouble(path + "/wave_height");

    auto steepness = db.ReadDouble(path + "/steepness");

    std::cout << "Regular wave T = " << param[0] << " s, Wave Height = "
              << param[1] << " m " << "steepness = " << steepness << std::endl;

    return param;

}

int main(int argc, char* argv[]) {

    std::cout << " ==================================================== \n"
                 " Benchmark test : Heave motion in regular wave \n"
                 " ==================================================== " << std::endl;

    // -- Input

    int iPeriod = 0;
    int iSteepness = 0;

    if (argv[1]) { iPeriod = atoi(argv[1]); }
    if (argv[2]) { iSteepness = atoi(argv[2]); }

    // -- System

    FrOffshoreSystem system;
    system.SetName("Sphere_RW");

    // -- Ocean

    auto ocean = system.GetEnvironment()->GetOcean();
    ocean->SetInfiniteDepth();
    ocean->SetDensity(1000.);

    // -- Wave field

    auto param = ReadParam("bench_sphere_regular.h5", iPeriod, iSteepness);

    double waveHeight = 0.5*param[1];
    double wavePeriod = param[0];

    auto waveField = ocean->GetFreeSurface()->SetAiryRegularWaveField();
    waveField->SetWaveHeight(waveHeight);
    waveField->SetWavePeriod(wavePeriod);
    waveField->SetDirection(NORTH(NWU), NWU, GOTO);

    // -- Body

    auto body = system.NewBody();
    body->SetName("Sphere");

    Position COGPosition(0., 0., -2.);
    FrFrame COGFrame(COGPosition, FrRotation(), NWU);

    body->SetPosition(Position(0., 0., 0.), NWU);

    body->GetDOFMask()->SetLock_X(true);
    body->GetDOFMask()->SetLock_Y(true);
    body->GetDOFMask()->SetLock_Rx(true);
    body->GetDOFMask()->SetLock_Ry(true);
    body->GetDOFMask()->SetLock_Rz(true);

    // -- Inertia

//    double mass = 2.618E5; // Theoretical mass.
    double mass = 2.61299E5; // Mass from mesh with 6200 faces.
//    double mass = 2.61488E5; // Mass from mesh with 10000 faces.

    double Ixx = 1.690E6;
    double Iyy = 1.690E6;
    double Izz = 2.606E6;

    FrInertiaTensor InertiaTensor(mass, Ixx, Iyy, Izz, 0., 0., 0., COGFrame, NWU);

    body->SetInertiaTensor(InertiaTensor);

    // -- Hydrodynamics

    auto hdb = make_hydrodynamic_database("sphere_hdb.h5");

    auto eqFrame = std::make_shared<FrEquilibriumFrame>(body.get());
    system.AddPhysicsItem(eqFrame);

    hdb->Map(0, body.get(), eqFrame);

    // -- Linear hydrostatics

    auto forceHst = make_linear_hydrostatic_force(hdb, body);

    // -- Radiation

    auto radiationModel = make_radiation_convolution_model(hdb, &system);
    radiationModel->SetImpulseResponseSize(body.get(), 6., 0.1);

    // -- Linear excitation

    auto excitationForce = make_linear_excitation_force(hdb, body);

    // -- Hydrodynamic mesh

//    auto bodyMesh = make_hydro_mesh_nonlinear(body,"Sphere_6200_faces.obj");
//    mathutils::Matrix33<double> Rotation;
//    Rotation.SetIdentity();
//    Position MeshOffset(0,0,0);
//    bodyMesh->SetMeshOffsetRotation(MeshOffset,Rotation);
//    bodyMesh->GetInitialMesh().Write("Mesh_Initial.obj");

    // -- Nonlinear hydrostatics

//    auto forceHst = make_nonlinear_hydrostatic_force(body,bodyMesh);
//    forceHst->SetLogged(true);

    // -- Nonlinear excitation

//    auto excitationForce = make_nonlinear_excitation_force(hdb, body,bodyMesh);
//    excitationForce->SetLogged(true);

    // -- Simulation

    auto dt = 0.005;

    system.SetTimeStep(dt);
    system.Initialize();

    auto time = -dt;

    std::vector<double> vtime;
    std::vector<double> heave;

    clock_t begin = clock();

    while (time < 200.) {
        time += dt;
        system.AdvanceTo(time);

        // ##CC
        //std::cout << "time : " << time << " ; position of the body = "
        //          << body->GetPosition(NWU).GetX() << " ; "
        //          << body->GetPosition(NWU).GetY() << " ; "
        //          << body->GetPosition(NWU).GetZ()
        //          << std::endl;

        std::cout << "time : " << time << " s" << std::endl;

        heave.push_back(body->GetPosition(NWU).GetZ());
        vtime.push_back(time);
        // ##CC
    }

    clock_t end = clock();
    double elapsed_secs = double(end - begin) / CLOCKS_PER_SEC;
    std::cout << elapsed_secs << std::endl;

    ValidationResults(vtime, heave, iPeriod, iSteepness);

    std::cout << " ================================= End ======================= " << std::endl;

}
