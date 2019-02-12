//
// Created by camille on 06/02/19.
//

#include "frydom/frydom.h"

using namespace frydom;

// ##CC Quick logging class

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
                << "Fe_X;Fe_Y;Fe_Z;Fe_MX;Fe_MY;Fe_MZ;"
                << "Fwd_X;Fwd_Y;Fwd_Z;Fwd_MX;Fwd_MY;Fwd_MZ;"
                << "Fittc57_X;Fittc57_Y;Fittc57_Z;Fittc57_MX;Fittc57_MY;Fittc57_MZ;"
                << "Fuser_pitch_X;Fuser_pitch_Y;Fuser_pitch_Z;Fuser_pitch_MX;Fuser_pitch_MY;Fuser_pitch_MZ;"
                << "Fuser_heave_X;Fuser_heave_Y;Fuser_heave_Z;Fuser_heave_MX;Fuser_heave_MY;Fuser_heave_MZ" << std::endl;

        outfile << "s;m;m;m;rad;rad;rad;"
                << "N;N;N;Nm;Nm;Nm;"
                << "N;N;N;Nm;Nm;Nm;"
                << "N;N;N;Nm;Nm;Nm;"
                << "N;N;N;Nm;Nm;Nm;"
                << "N;N;N;Nm;Nm;Nm;"
                << "N;N;N;Nm;Nm;Nm;"
                << std::endl;
    }

    void Write(double time, Position bodyPosition, FrRotation_ bodyRotation,
               Force Fh_force, Torque Fh_torque, Force Fe_force, Torque Fe_torque, Force Fwd_force, Torque Fwd_torque,
               Force Fittc_force, Torque Fittc_torque, Force Fuser_pitch_force, Torque Fuser_pitch_torque,
               Force Fuser_heave_force, Torque Fuser_heave_torque) {

        double Rx, Ry, Rz;
        bodyRotation.GetCardanAngles_RADIANS(Rx, Ry, Rz, NWU);

        outfile << time << ";"
                << bodyPosition.GetX() << ";" << bodyPosition.GetY() << ";" << bodyPosition.GetZ() << ";"
                << Rx << ";" << Ry << ";" << Rz << ";"
                << Fh_force.GetFx() << ";" << Fh_force.GetFy() << ";" << Fh_force.GetFz() << ";"
                << Fh_torque.GetMx() << ";" << Fh_torque.GetMy() << ";" << Fh_torque.GetMz() << ";"
                << Fe_force.GetFx() << ";" << Fe_force.GetFy() << ";" << Fe_force.GetFz() << ";"
                << Fe_torque.GetMx() << ";" << Fe_torque.GetMy() << ";" << Fe_torque.GetMz() << ";"
                << Fwd_force.GetFx() << ";" << Fwd_force.GetFy() << ";" << Fwd_force.GetFz() << ";"
                << Fwd_torque.GetMx() << ";" << Fwd_torque.GetMy() << ";" << Fwd_torque.GetMz() << ";"
                << Fittc_force.GetFx() << ";" << Fittc_force.GetFy() << ";" << Fittc_force.GetFz() << ";"
                << Fittc_torque.GetMx() << ";" << Fittc_torque.GetMy() << ";" << Fittc_torque.GetMz() << ";"
                << Fuser_pitch_force.GetFx() << ";" << Fuser_pitch_force.GetFy() << ";" << Fuser_pitch_force.GetFz() << ";"
                << Fuser_pitch_torque.GetMx() << ";" << Fuser_pitch_torque.GetMy() << ";" << Fuser_pitch_torque.GetMz() << ";"
                << Fuser_heave_force.GetFx() << ";" << Fuser_heave_force.GetFy() << ";" << Fuser_heave_force.GetFz() << ";"
                << Fuser_heave_torque.GetMx() << ";" << Fuser_heave_torque.GetMy() << ";" << Fuser_heave_torque.GetMz()
                << std::endl;
    };

    void Close() {
        outfile.close();
    }
}; // ##CC logging


// ----------------------------------------------------------
// Steady Pitch Torque
// ----------------------------------------------------------

class SteadyPitchTorque : public FrForce_ {

public:

    void Update(double time) override {

        auto speed = m_body->GetVelocityInWorld(NWU).GetVx();
        auto torque = 4.332 * std::pow(speed, 6)
                    - 9.1135 * std::pow(speed, 5)
                    - 9.7756 * std::pow(speed, 4)
                    + 34.232 * std::pow(speed, 3)
                    - 22.7359 * std::pow(speed, 2);

        SetTorqueInBodyAtCOG(Torque(0., -torque, 0.), NWU);

        return ;
    }

    void StepFinalize() override {};
};

// ----------------------------------------------------------
// Steady Heave Force
// ----------------------------------------------------------

class SteadyHeaveForce : public FrForce_ {

public:

    void Update(double time) override {

        auto speed = m_body->GetVelocityInWorld(NWU).GetVx();

        auto force = -12.32426 * std::pow(speed, 3)
                   - 2.8696 * std::pow(speed, 2);

        SetForceInWorldAtCOG(Force(0., 0., force), NWU);

        return;
    }

    void StepFinalize() override {}
};

// -----------------------------------------------------------
// ITTC57 : residual coefficient
// -----------------------------------------------------------

double ResidualITTC(double speed) {

    if (std::abs(speed - 1.04) < 1E-2) {
        return 5.3696e-4;
    } else if (std::abs(speed - 1.532) < 1E-2) {
        return 9.0677e-4;
    } else if (std::abs(speed - 1.86) < 1E-2) {
        return 1.6812e-3;
    } else if (std::abs(speed - 2.243) < 1E-2) {
        return 4.02529e-3;
    } else {
        std::cout << "warning : no residual coefficient for this speed value" << std::endl;
        std::cout << "        : residual set to 0. " << std::endl;
    }
    return 0.;
};


// ------------------------------------------------------------
// Benchmark : main
// ------------------------------------------------------------

int main(int argc, char* argv[]) {

    std::cout << " ======================================================= \n"
                 " Benchmark test : DTMB 5512 captive motion \n"
                 " =======================================================" << std::endl;

    // -- Input

    double speed = atof(argv[1]);
    double ak = 0.5*atof(argv[2]);
    double Tk = atof(argv[3]);
    std::string name = argv[4];

    bool captive_test = true;

    // -- System

    FrOffshoreSystem_ system;

    // -- Ocean

    auto ocean = system.GetEnvironment()->GetOcean();
    ocean->GetSeabed()->SetBathymetry(-3.048, NWU);
    ocean->SetDensity(1000.);

    auto waveField = ocean->GetFreeSurface()->SetAiryRegularWaveField();
    waveField->SetWaveHeight(ak);
    waveField->SetWavePeriod(Tk);
    waveField->SetDirection(180., DEG, NWU, GOTO);

    // -- Body

    auto body = system.NewBody();

    Position COGPosition(0., 0., 0.03);
    FrFrame_ COGFrame(COGPosition, FrRotation_(), NWU);

    body->SetPosition(Position(0., 0., 0.), NWU);
    body->SetVelocityInBodyNoRotation(Velocity(speed, 0., 0.) ,NWU);

    body->GetDOFMask()->SetLock_Y(true);
    body->GetDOFMask()->SetLock_Rx(true);
    body->GetDOFMask()->SetLock_Rz(true);

    if (captive_test) {
        body->GetDOFMask()->SetLock_Z(true);
        body->GetDOFMask()->SetLock_Ry(true);
    }

    // -- Inertia

    double mass = 86.0;

    double Ixx = 1.98;
    double Iyy = 53.88;
    double Izz = 49.99;

    FrInertiaTensor_ InertiaTensor(mass, Ixx, Iyy, Izz, 0., 0., 0., COGFrame, NWU);

    body->SetInertiaTensor(InertiaTensor);

    // -- Hydrodynamics

    auto hdb = make_hydrodynamic_database("DTMB5512.hdb5");

    auto eqFrame = std::make_shared<FrEquilibriumFrame_>(body.get());
    eqFrame->InitSpeedFromBody(true);
    system.AddPhysicsItem(eqFrame);

    hdb->Map(0, body.get(), eqFrame);

    // -- Hydrostatic

    //auto forceHst = std::make_shared<FrLinearHydrostaticForce_>(hdb);
    //body->AddExternalForce(forceHst);
    auto forceHst = make_linear_hydrostatic_force(hdb, body);

    // -- Radiation

    auto radiationModel = make_radiation_convolution_model(hdb, &system);

    radiationModel->SetImpulseResponseSize(body.get(), 8., 0.01);

    // -- Excitation

    auto excitationForce = make_linear_excitation_force(hdb, body);

    // -- Wave Drift force

    auto waveDriftForce = std::make_shared<FrWaveDriftForce_>(hdb);
    body->AddExternalForce(waveDriftForce);

    // -- ITTC57

    auto lpp = 3.048;
    auto wettedSurfaceArea = 1.371;

    auto ct = ResidualITTC(speed);
    auto forceResistance = std::make_shared<FrITTCResistance_>(lpp, wettedSurfaceArea, ct, 0.03);
    body->AddExternalForce(forceResistance);

    // -- Steady force

    auto forcePitch = std::make_shared<SteadyPitchTorque>();
    body->AddExternalForce(forcePitch);

    auto forceHeave = std::make_shared<SteadyHeaveForce>();
    body->AddExternalForce(forceHeave);

    // -- Simulation

    auto dt = 0.008;

    system.SetTimeStep(dt);
    system.Initialize();

    double time = 0.;


    // ##CC
    Logging log;
    log.Open(name);
    // ##CC

    while (time < 40.) {
        time += dt;
        body->SetVelocityInBodyNoRotation(Velocity(speed, 0., 0.) ,NWU);
        system.AdvanceTo(time);

        // ##CC
        std::cout << "time : " << time << " ; position of the body = "
                  << body->GetPosition(NWU).GetX() << " ; "
                  << body->GetPosition(NWU).GetY() << " ; "
                  << body->GetPosition(NWU).GetZ()
                  << std::endl;

        std::cout << " Position of the Equilibrium frame : "
                  << eqFrame->GetPosition(NWU).GetX() << ";"
                  << eqFrame->GetPosition(NWU).GetY() << ";"
                  << eqFrame->GetPosition(NWU).GetZ() << std::endl;


        log.Write(time,
            body->GetPosition(NWU), body->GetRotation(),
            forceHst->GetForceInWorld(NWU),forceHst->GetTorqueInBodyAtCOG(NWU),
            excitationForce->GetForceInWorld(NWU), excitationForce->GetTorqueInBodyAtCOG(NWU),
            waveDriftForce->GetForceInWorld(NWU), waveDriftForce->GetTorqueInBodyAtCOG(NWU),
            forceResistance->GetForceInWorld(NWU), forceResistance->GetTorqueInBodyAtCOG(NWU),
            forcePitch->GetForceInWorld(NWU), forcePitch->GetTorqueInBodyAtCOG(NWU),
            forceHeave->GetForceInWorld(NWU), forceHeave->GetTorqueInBodyAtCOG(NWU)
        );
        // ##CC

    }

    log.Close();

    std::cout << "=============================== End ========================" << std::endl;

}