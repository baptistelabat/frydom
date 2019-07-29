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

    std::cout << " ===================================================== \n"
                 " Benchmark test : Nonlinear hydrostatics on a box \n"
                 " ===================================================== " << std::endl;

//    cppfs::FilePath resources_path(std::string(RESOURCES_PATH));
    cppfs::FilePath resources_path("/home/lletourn/Documents/DEV/frydom/tests/data/bench/box");

    // -- System

    FrOffshoreSystem system;
    system.SetName("NLHSBox");
    system.GetPathManager()->SetLogFrameConvention(NWU);

    auto Ocean = system.GetEnvironment()->GetOcean();
    Ocean->SetDensity(1023.);

    // -- Body

    auto body = system.NewBody();
    body->SetName("Box");

    auto mass = 0.5 * 8. * 4. * 2. * system.GetEnvironment()->GetFluidDensity(WATER);
    makeItBox(body,8,4,2,mass);

    bool linear = false;
    if (linear) {
        // -- Linear hydrostatics
        auto eqFrame = std::make_shared<FrEquilibriumFrame>(body.get());
        system.AddPhysicsItem(eqFrame);
        auto forceHst = make_linear_hydrostatic_force(eqFrame, body, resources_path.resolve("box_250.obj").path(), FrFrame());
    } else {
        // Nonlinear hydrostatics
        auto bodyMesh = make_hydro_mesh(body,resources_path.resolve("box_250.obj").path(),FrFrame(),FrHydroMesh::ClippingSupport::PLANESURFACE);
        //bodyMesh->GetInitialMesh().Write("Mesh_Initial.obj");
        auto forceHst = make_nonlinear_hydrostatic_force(body,bodyMesh);
    }
    // -- Simulation

    auto dt = 0.01;

    system.SetTimeStep(dt);
    system.Initialize();

    // Decay test initial position.
    FrRotation decayRot; //decayRot.RotX_DEGREES(10., NWU);
    decayRot.SetCardanAngles_DEGREES(2.,0.,0.,NWU);
//    body->SetPosition(Position(0., 0., 0.5), NWU);
    body->SetRotation(decayRot);


//    system.RunInViewer(10.);




    auto time = 0.;

    clock_t begin = clock();

    while (time < 200.) {
        time += dt;
        system.AdvanceTo(time);
        std::cout << "time : " << time << std::endl;
    }

    clock_t end = clock();
    double elapsed_secs = double(end - begin) / CLOCKS_PER_SEC;
    std::cout << "Elapsed cpu time in seconds : " << elapsed_secs << std::endl;
    std::cout << "============================== End ===================================== " << std::endl;

} // end namespace frydom
