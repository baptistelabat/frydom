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

class TestBody : public FrBody {

    void StepFinalize() override {

        double L, B, H;
        L = H = B = 5.;
//            std::cout<<"t = "<<time<<", c = "<<c<<std::endl;
        auto mass = L * H * B * GetDensity() * GetSystem()->GetEnvironment()->GetFluidDensity(WATER);

        // Properties of the box
        double xSize2 = L * L;
        double ySize2 = H * H;
        double zSize2 = B * B;

        // inertia
        double Ixx = (1./12.) * mass * (ySize2 + zSize2);
        double Iyy = (1./12.) * mass * (xSize2 + zSize2);
        double Izz = (1./12.) * mass * (xSize2 + ySize2);

        // Building the chrono body
        SetInertiaTensor(FrInertiaTensor(mass, Ixx, Iyy, Izz, 0., 0., 0., Position(), NWU));


        FrBody::StepFinalize();
    }

    double GetDensity() const {

        if (GetSystem()->GetTime()>200.) return 0.9;

        return 0.1 + 0.8 * GetSystem()->GetTime()/200.;
    }

    void AddFields() override {

        FrBody::AddFields();

        m_message->AddField<double>("density", " ", "density of the body, relatively to water",
                                    [this]() { return GetDensity(); });

    }


};

int main(int argc, char* argv[]) {

    std::cout << " ===================================================== \n"
                 " Benchmark test : Nonlinear hydrostatics on a box \n"
                 " ===================================================== \n" << std::endl;

    cppfs::FilePath resources_path(std::string(RESOURCES_PATH));

    // -- System

    FrOffshoreSystem system;
//    system.SetLogged(false);
    system.SetName("NLHSBox");
    system.GetPathManager()->SetLogFrameConvention(NWU);

    auto Ocean = system.GetEnvironment()->GetOcean();
    Ocean->SetDensity(1023.);

    // -- Body

//    auto body = system.NewBody();
    auto body = std::make_shared<TestBody>();
    system.AddBody(body);

    body->SetName("Box");

    double L, B, H, c;
    L = H = B = 5.; c = 0.1;
//    L = 8; B = 4; H = 2; c = 0.5;

    auto mass = L * H * B * c * system.GetEnvironment()->GetFluidDensity(WATER);
    makeItBox(body,L,B,H,mass);

    body->RemoveAssets();

    bool linear = false;
    if (linear) {
        // -- Linear hydrostatics
        auto eqFrame = make_equilibrium_frame("EqFrame", &system, body);

        auto forceHst = make_linear_hydrostatic_force(eqFrame, body, resources_path.resolve("box_385.obj").path(), FrFrame());
    } else {
        // Nonlinear hydrostatics
        auto bodyMesh = make_hydro_mesh(body,resources_path.resolve("box_385.obj").path(),FrFrame(),FrHydroMesh::ClippingSupport::PLANESURFACE);
        //bodyMesh->GetInitialMesh().Write("Mesh_Initial.obj");
        auto forceHst = make_nonlinear_hydrostatic_force(body,bodyMesh);
    }


    auto dampingForce = make_linear_damping_force(body, WATER, true);
    dampingForce->SetDiagonalDamping(1E4,1E4,1E4,1E5,1E5,1E5);

    // -- Simulation

    auto dt = 0.01;

    system.SetTimeStep(dt);
    system.Initialize();

    // Decay test initial position.
    FrRotation decayRot;
    decayRot.SetCardanAngles_DEGREES(20.,0.,0.,NWU);
    body->SetPosition(Position(0., 0., 2.), NWU);
    body->SetRotation(decayRot);


    system.RunInViewer(250., 10, true, 20);


//    auto time = 0.;
//
//    clock_t begin = clock();
//
//    while (time < 1000.) {
//        time += dt;
//
//        system.AdvanceTo(time);
//        std::cout << "time : " << time << std::endl;
//    }
//
//    clock_t end = clock();
//    double elapsed_secs = double(end - begin) / CLOCKS_PER_SEC;
//    std::cout << "Elapsed cpu time in seconds : " << elapsed_secs << std::endl;
//    std::cout << "============================== End ===================================== " << std::endl;

} // end namespace frydom
