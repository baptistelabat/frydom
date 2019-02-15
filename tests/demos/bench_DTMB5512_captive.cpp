//
// Created by camille on 06/02/19.
//

#include "frydom/frydom.h"

using namespace frydom;



// ##CC Quick logging class

class AddedMassRadiationForce {

protected :
    FrHydroDB_* m_HDB;
    GeneralizedForce m_force;
    FrBody_* m_body;

public:

    AddedMassRadiationForce(FrHydroDB_* HDB, FrBody_* body) : m_HDB(HDB), m_body(body) {}

    void Update(FrBody_* body) {

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
                << "Fuser_heave_X;Fuser_heave_Y;Fuser_heave_Z;Fuser_heave_MX;Fuser_heave_MY;Fuser_heave_MZ;"
                << "Frad_conv_X;Frad_conv_Y;Frad_conv_Z;Frad_conv_MX;Frad_conv_MY;Frad_conv_MZ;"
                << "Ftot_X;Ftot_Y;Ftot_Z;Ftot_MX;Ftot_MY;Ftot_MZ;"
                << "Frad_Minf_X;Frad_Minf_Y;Frad_Minf_Z;Frad_Minf_MX;Frad_Minf_MY;Frad_Minf_MZ"
                << std::endl;

        outfile << "s;m;m;m;rad;rad;rad;"
                << "N;N;N;Nm;Nm;Nm;"
                << "N;N;N;Nm;Nm;Nm;"
                << "N;N;N;Nm;Nm;Nm;"
                << "N;N;N;Nm;Nm;Nm;"
                << "N;N;N;Nm;Nm;Nm;"
                << "N;N;N;Nm;Nm;Nm;"
                << "N;N;N;Nm;Nm;Nm;"
                << "N;N;N;Nm;Nm;Nm;"
                << "N;N;N;Nm;Nm;Nm"
                << std::endl;
    }

    void Write(double time, Position bodyPosition, FrRotation_ bodyRotation,
               Force Fh_force, Torque Fh_torque,
               Force Fe_force, Torque Fe_torque,
               Force Fwd_force, Torque Fwd_torque,
               Force Fittc_force, Torque Fittc_torque,
               Force Fuser_pitch_force, Torque Fuser_pitch_torque,
               Force Fuser_heave_force, Torque Fuser_heave_torque,
               Force Frad_conv_force, Torque Frad_conv_torque,
               Force Ftot_force, Torque Ftot_torque,
               Force Frad_Minf_force, Torque Frad_Minf_torque ) {

        double Rx, Ry, Rz;
        bodyRotation.GetCardanAngles_RADIANS(Rx, Ry, Rz, NWU);

        Force Fdelta_force = Ftot_force
                            - Fh_force - Fe_force - Fwd_force - Fittc_force
                            - Fuser_pitch_force - Fuser_heave_force - Frad_conv_force;

        Torque Fdelta_torque = Ftot_torque
                           - Fh_torque - Fe_torque - Fwd_torque - Fittc_torque
                           - Fuser_pitch_torque - Fuser_heave_torque - Frad_conv_torque;

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
                << Fuser_heave_torque.GetMx() << ";" << Fuser_heave_torque.GetMy() << ";" << Fuser_heave_torque.GetMz() << ";"
                << Frad_conv_force.GetFx() << ";" << Frad_conv_force.GetFy() << ";" << Frad_conv_force.GetFz() << ";"
                << Frad_conv_torque.GetMx() << ";" << Frad_conv_torque.GetMy() << ";" << Frad_conv_torque.GetMz() << ";"
                << Ftot_force.GetFx() << ";" << Ftot_force.GetFy() << ";" << Ftot_force.GetFz() << ";"
                << Ftot_torque.GetMx() << ";" << Ftot_torque.GetMy() << ";" << Ftot_torque.GetMz() << ";"
                << Frad_Minf_force.GetFx() << ";" << Frad_Minf_force.GetFy() << ";" << Frad_Minf_force.GetFz() << ";"
                << Frad_Minf_torque.GetMx() << ";" << Frad_Minf_torque.GetMy() << ";" << Frad_Minf_torque.GetMz()
                << std::endl;
    };

    void Close() {
        outfile.close();
    }
}; // ##CC logging


// ##CC impose speed

Velocity BodyVelocity(double time, double period, double amplitude) {

    double omega = 2.*M_PI / period;
    return Velocity(0., 0., amplitude * cos(omega*time));
}

AngularVelocity BodyAngularVelocity(double time, double period, double amplitude) {

    double omega = 2.*M_PI / period;
    double ak = amplitude * M_PI / 180.;
    return AngularVelocity(0., ak* sin(omega*time), 0.);
}


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

    bool captive_test = false;

    // -- System

    FrOffshoreSystem_ system;

    // -- Ocean
    auto ocean = system.GetEnvironment()->GetOcean();
    ocean->GetSeabed()->SetBathymetry(-3.048, NWU);
    ocean->SetDensity(1000.);

    auto waveField = ocean->GetFreeSurface()->SetAiryRegularWaveField();
    waveField->SetWaveHeight(ak); // ak
    waveField->SetWavePeriod(Tk); // Tk
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

    // ##CC
    body->GetDOFMask()->SetLock_X(true);
    //body->GetDOFMask()->SetLock_Z(true);
    //body->GetDOFMask()->SetLock_Ry(true);
    // ##CC

    if (captive_test) {
        body->GetDOFMask()->SetLock_Z(true);
        body->GetDOFMask()->SetLock_Ry(true);
    } else {
        //body->GetDOFMask()->SetLock_X(true);
        //body->GetDOFMask()->SetLock_Ry(true);
        system.GetEnvironment()->GetTimeRamp()->Activate();
        system.GetEnvironment()->GetTimeRamp()->SetDuration(20.);
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
    //eqFrame->InitSpeedFromBody(true);
    system.AddPhysicsItem(eqFrame);

    hdb->Map(0, body.get(), eqFrame);

    // -- Hydrostatic

    //auto forceHst = std::make_shared<FrLinearHydrostaticForce_>(hdb);
    //body->AddExternalForce(forceHst);
    auto forceHst = make_linear_hydrostatic_force(hdb, body);

    // -- Radiation

    auto radiationModel = make_radiation_convolution_model(hdb, &system);

    auto radiationForce = std::make_shared<FrRadiationConvolutionForce_>(radiationModel.get());
    body->AddExternalForce(radiationForce);

    radiationModel->SetImpulseResponseSize(body.get(), 10., 0.01);

    // ##CC for monitoring
    auto radiationAddedMassForce = std::make_shared<AddedMassRadiationForce>(hdb.get(), body.get());
    // ##CC

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

        // ##CC test user define velocity
        //body->SetVelocityInBodyNoRotation(Velocity(speed, 0., 0.) ,NWU);
        //body->SetVelocityInBodyNoRotation(BodyVelocity(time, Tk, 2.*ak), NWU);
        //body->SetCOGAngularVelocityInWorld(BodyAngularVelocity(time, Tk, 10.), NWU);
        // ##CC

        system.AdvanceTo(time);

        // ##CC monitoring
        std::cout << "time : " << time << " ; position of the body = "
                  << body->GetPosition(NWU).GetX() << " ; "
                  << body->GetPosition(NWU).GetY() << " ; "
                  << body->GetPosition(NWU).GetZ()
                  << std::endl;

        std::cout << " velocity of the body = "
                  << body->GetVelocityInWorld(NWU).GetVx() << ";"
                  << body->GetVelocityInWorld(NWU).GetVy() << ";"
                  << body->GetVelocityInWorld(NWU).GetVz()
                  << std::endl;

        //std::cout << " Position of the Equilibrium frame : "
        //          << eqFrame->GetPosition(NWU).GetX() << ";"
        //          << eqFrame->GetPosition(NWU).GetY() << ";"
        //          << eqFrame->GetPosition(NWU).GetZ() << std::endl;

        radiationAddedMassForce->Update(body.get());

        log.Write(time,
            body->GetPosition(NWU), body->GetRotation(),
            forceHst->GetForceInWorld(NWU),forceHst->GetTorqueInBodyAtCOG(NWU),
            excitationForce->GetForceInWorld(NWU), excitationForce->GetTorqueInBodyAtCOG(NWU),
            waveDriftForce->GetForceInWorld(NWU), waveDriftForce->GetTorqueInBodyAtCOG(NWU),
            forceResistance->GetForceInWorld(NWU), forceResistance->GetTorqueInBodyAtCOG(NWU),
            forcePitch->GetForceInWorld(NWU), forcePitch->GetTorqueInBodyAtCOG(NWU),
            forceHeave->GetForceInWorld(NWU), forceHeave->GetTorqueInBodyAtCOG(NWU),
            radiationForce->GetForceInWorld(NWU), radiationForce->GetTorqueInBodyAtCOG(NWU),
            body->GetTotalForceInWorld(NWU), body->GetTotalTorqueInBodyAtCOG(NWU),
            radiationAddedMassForce->GetForceInWorld(), radiationAddedMassForce->GetTorqueInWorldAtCOG()
        );
        // ##CC

    }

    log.Close();

    std::cout << "=============================== End ========================" << std::endl;

}