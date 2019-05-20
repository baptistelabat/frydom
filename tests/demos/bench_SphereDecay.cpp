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

// ##CC Quick logging class

class FrNullForce : public FrForce {

    void Compute(double time) override { }

public :

    void Initialize() override {}
};


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
        acc.SetAcceleration(body->GetCOGAccelerationInWorld(NWU));
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

Velocity BodyVelocity(double time, double period, double amplitude) {

    double omega = 2.*M_PI / period;
    return Velocity(0., 0., amplitude * cos(omega*time));
}


class Logging {

protected:
    std::fstream outfile;

public:

    Logging() = default;

    ~Logging() { outfile.close(); }

    void Open(std::string filename) {

        std::cout << "open file : " << filename << std::endl;

        outfile.open(filename + ".csv", std::fstream::out);
        outfile << "time;x;y;z;rx;ry;rz;"
                << "Fh_X;Fh_Y;Fh_Z;Fh_MX;Fh_MY;Fh_MZ;"
                << "Frad_conv_X;Frad_conv_Y;Frad_conv_Z;Frad_conv_MX;Frad_conv_MY;Frad_conv_MZ;"
                << "Ftot_X;Ftot_Y;Ftot_Z;Ftot_MX;Ftot_MY;Ftot_MZ;"
                << "Frad_Minf_X;Frad_Minf_Y;Frad_Minf_Z;Frad_Minf_MX;Frad_Minf_MY;Frad_Minf_Mz"
                << std::endl;

        outfile << "s;m;m;m;rad;rad;rad;"
                << "N;N;N;Nm;Nm;Nm;"
                << "N;N;N;Nm;Nm;Nm;"
                << "N;N;N;Nm;Nm;Nm;"
                << "N;N;N;Nm;Nm;Nm"
                << std::endl;
    }

    void Write(double time, Position bodyPosition, FrRotation bodyRotation,
               Force Fh_force, Torque Fh_torque,
               Force Frad_conv_force, Torque Frad_conv_torque,
               Force Frad_minf_force, Torque Frad_minf_torque,
               Force Ftot_force, Torque Ftot_torque) {

        double Rx, Ry, Rz;
        bodyRotation.GetCardanAngles_RADIANS(Rx, Ry, Rz, NWU);

        Force Fdelta_force = Ftot_force - Fh_force - Frad_conv_force - Force(0., 0., -2568258.);
        Torque Fdelta_torque = Ftot_torque - Fh_torque - Frad_conv_torque;

        outfile << time << ";"
                << bodyPosition.GetX() << ";" << bodyPosition.GetY() << ";" << bodyPosition.GetZ() << ";"
                << Rx << ";" << Ry << ";" << Rz << ";"
                << Fh_force.GetFx() << ";" << Fh_force.GetFy() << ";" << Fh_force.GetFz() << ";"
                << Fh_torque.GetMx() << ";" << Fh_torque.GetMy() << ";" << Fh_torque.GetMz() << ";"
                << Frad_conv_force.GetFx() << ";" << Frad_conv_force.GetFy() << ";" << Frad_conv_force.GetFz() << ";"
                << Frad_conv_torque.GetMx() << ";" << Frad_conv_torque.GetMy() << ";" << Frad_conv_torque.GetMz() << ";"
                << Ftot_force.GetFx() << ";" << Ftot_force.GetFy() << ";" << Ftot_force.GetFz() << ";"
                << Ftot_torque.GetMx() << ";" << Ftot_torque.GetMy() << ";" << Ftot_torque.GetMz() << ";"
                << Frad_minf_force.GetFx() << ";" << Frad_minf_force.GetFy() << ";" << Frad_minf_force.GetFz() << ";"
                << Frad_minf_torque.GetMx() << ";" << Frad_minf_torque.GetMy() << ";" << Frad_minf_torque.GetMz()
                << std::endl;
    };

    void Close() {
        outfile.close();
    }
}; // ##CC logging


int main(int argc, char* argv[]) {

    std::cout << " ===================================================== \n"
                 " Benchmark test : Sphere Decay \n"
                 " ===================================================== " << std::endl;

    // -- System

    FrOffshoreSystem system;
    system.SetName("Sphere_Decay");

    auto Ocean = system.GetEnvironment()->GetOcean();
    Ocean->SetDensity(1000);

    // -- Body

    auto body = system.NewBody();
    body->SetName("Sphere");

    Position COGPosition(0., 0., -2.);
    FrFrame COGFrame(COGPosition, FrRotation(), NWU);

    body->SetPosition(Position(0., 0., 0.), NWU);

    // -- Inertia

    double mass = 2.618E5;

    double Ixx  = 1.690E6;
    double Iyy  = 1.690E6;
    double Izz  = 2.606E6;

    FrInertiaTensor InertiaTensor(mass, Ixx, Iyy, Izz, 0., 0., 0., COGFrame, NWU);

    body->SetInertiaTensor(InertiaTensor);

    body->GetDOFMask()->SetLock_X(true);
    body->GetDOFMask()->SetLock_Y(true);
    body->GetDOFMask()->SetLock_Rx(true);
    body->GetDOFMask()->SetLock_Ry(true);
    body->GetDOFMask()->SetLock_Rz(true);

    // -- Hydrodynamics

    auto hdb = make_hydrodynamic_database("sphere_hdb.h5");

    auto eqFrame = std::make_shared<FrEquilibriumFrame>(body.get());
    system.AddPhysicsItem(eqFrame);

    hdb->Map(0, body.get(), eqFrame);

    // -- Linear hydrostatics

    auto forceHst = make_linear_hydrostatic_force(hdb, body);

    // Nonlinear hydrostatics
    //auto bodyMesh = make_hydro_mesh(body,"Sphere_10000_faces.obj",FrFrame(),FrHydroMesh::ClippingSupport::WAVESURFACE);
    //bodyMesh->GetInitialMesh().Write("Mesh_Initial.obj");

    //auto forceHst = make_nonlinear_hydrostatic_force(body,bodyMesh);

    // -- Radiation

    auto radiationModel = make_radiation_convolution_model(hdb, &system);

    auto radiationForce = std::make_shared<FrRadiationConvolutionForce>(radiationModel.get());
    //auto radiationForce = std::make_shared<FrNullForce>();
//    body->AddExternalForce(radiationForce);

    radiationModel->SetImpulseResponseSize(body.get(), 6., 0.01);

    // ##CC for monitoring
    auto radiationAddedMassForce = std::make_shared<AddedMassRadiationForce>(hdb.get(), body.get());
    // ##CC

    // -- Simulation

//    auto dt = 0.005;
    auto dt = 0.01;

    system.SetTimeStep(dt);
    system.Initialize();

    // Decay test position.
    body->SetPosition(Position(0., 0., 4.99), NWU);

    auto time = 0.;

    // ##CC
    //Logging log;
    //log.Open("sphere");
    // ##CC

    clock_t begin = clock();

    while (time < 40.) {

        time += dt;
        system.AdvanceTo(time);

        std::cout << "time : " << time << " ; position of the body = "
                  << body->GetPosition(NWU).GetX() << " ; "
                  << body->GetPosition(NWU).GetY() << " ; "
                  << body->GetPosition(NWU).GetZ()
                  << std::endl;

        radiationAddedMassForce->Update(body.get());

        //log.Write(time,
        //          body->GetPosition(NWU), body->GetRotation(),
        //          forceHst->GetForceInWorld(NWU),forceHst->GetTorqueInBodyAtCOG(NWU),
        //          radiationForce->GetForceInWorld(NWU), radiationForce->GetTorqueInBodyAtCOG(NWU),
        //          radiationAddedMassForce->GetForceInWorld(), radiationAddedMassForce->GetTorqueInWorldAtCOG(),
        //          body->GetTotalExtForceInWorld(NWU), body->GetTotalTorqueInBodyAtCOG(NWU)
        //);

    }

    clock_t end = clock();
    double elapsed_secs = double(end - begin) / CLOCKS_PER_SEC;
    std::cout << elapsed_secs << std::endl;
    std::cout << "============================== End ===================================== " << std::endl;

} // end namespace frydom
