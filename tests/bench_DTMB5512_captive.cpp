//
// Created by camille on 12/07/18.
//

#include <vector>

#include "frydom/frydom.h"
#include "matplotlibcpp.h"

using namespace frydom;
using namespace chrono;

void PlotData(std::vector<double>& vtime, std::vector<double>& ydata, std::string ylabel) {

    matplotlibcpp::plot(vtime, ydata);
    matplotlibcpp::xlabel("time (s)");
    matplotlibcpp::ylabel(ylabel);

}

void PlotResult(std::vector<double>& vtime, std::vector<ChVector<double>>& vposition,
                std::vector<ChVector<double>>& vrotation, int idata) {

    std::vector<double> x, y, z;
    std::vector<double> rx, ry, rz;

    for(auto position: vposition) {
        x.push_back(position.x());
        y.push_back(position.y());
        z.push_back(position.z());
    }

    for (auto rotation: vrotation) {
        rx.push_back(rotation.x());
        ry.push_back(rotation.y());
        rz.push_back(rotation.z());
    }

    std::string ylabel1, ylabel2;

    switch (idata) {
        case 0:
            ylabel1 = "position (m)";
            ylabel2 = "rotation (rad)";
        case 1:
            ylabel1 = "force (N)";
            ylabel2 = "torque (N.m)";
    }
    matplotlibcpp::subplot(2,3,1);
    PlotData(vtime, x, ylabel1);
    matplotlibcpp::title("surge");

    matplotlibcpp::subplot(2,3,2);
    PlotData(vtime, y, ylabel1);
    matplotlibcpp::title("sway");

    matplotlibcpp::subplot(2,3,3);
    PlotData(vtime, z, ylabel1);
    matplotlibcpp::title("heave");

    matplotlibcpp::subplot(2,3,4);
    PlotData(vtime, rx, ylabel2);
    matplotlibcpp::title("roll");

    matplotlibcpp::subplot(2,3,5);
    PlotData(vtime, ry, ylabel2);
    matplotlibcpp::title("pitch");

    matplotlibcpp::subplot(2,3,6);
    PlotData(vtime, rz, ylabel2);
    matplotlibcpp::title("yaw");

    matplotlibcpp::show();
}


// ----------------------------------------------------------
// Ship model : DTMB5512 (IIHR - Yoon 2009)
// ----------------------------------------------------------

std::shared_ptr<FrShip> DTMB5512(FrOffshoreSystem* system) {

    auto ship_pos = ChVector<>(0., 0., 0.);

    auto ship = std::make_shared<FrShip>();
    system->AddBody(ship);

    // Geometry properties
    ship->SetName("DTMB5512");
    ship->SetHydroMesh("DTMB5512.obj", true);
    ship->SetLpp(3.048);
    ship->SetMass(86.0);
    ship->SetCOG(ChVector<>(0., 0., -0.036));
    ship->SetInertiaXX(ChVector<>(1.98, 53.88, 49.99));
    ship->SetPos(ChVector<>(0., 0., 0.));
    ship->SetEquilibriumFrame(MeanMotion, 60.);

    // Hydrostatics
    auto hstForce = std::make_shared<FrLinearHydrostaticForce>();
    auto hstStiffness = hstForce->GetStiffnessMatrix();
    hstStiffness->SetDiagonal(9.68e3, 8.97e1, 5.48e3);
    ship->AddForce(hstForce);

    // Hydrodynamics
    system->SetHydroDB("DTMB5512_hdb.h5");
    auto hydroMapIndex = system->GetHydroMapNb()-1;
    system->GetHydroMapper(hydroMapIndex)->Map(ship, 0);

    // Radiation model
    auto radModel = std::make_shared<FrRadiationConvolutionModel>(system->GetHydroDB(hydroMapIndex), system);
    radModel->SetHydroMapIndex(hydroMapIndex); // TODO : patch hydro map multibody
    radModel->AddRadiationForceToHydroBody(ship);

    // Wave Probe
    auto waveField = system->GetEnvironment()->GetFreeSurface()->GetLinearWaveField();
    //auto waveProbe = waveField->NewWaveProbe(ship_pos.x(), ship_pos.y());
    auto waveProbe = std::make_shared<FrLinearWaveProbe>();
    waveProbe->AttachedNode(ship->GetEquilibriumFrame());
    waveField->AddWaveProbe(waveProbe);
    waveProbe->Initialize();

    // Wave Excitation force
    auto excForce = std::make_shared<FrLinearExcitationForce>();
    ship->AddForce(excForce);
    excForce->SetWaveProbe(waveProbe);
    excForce->SetHydroMapIndex(hydroMapIndex);

    // Standard current viscous drag force model
    auto drag_force = std::make_shared<FrCurrentStandardForce>(ship);
    ship->AddForce(drag_force);

    return ship;
}


int main(int argc, char* argv[]) {

    // ------------------------------------------------------
    // System
    // ------------------------------------------------------

    FrOffshoreSystem system;

    // ------------------------------------------------------
    // Environment
    // ------------------------------------------------------

    system.GetEnvironment()->GetFreeSurface()->SetLinearWaveField(LINEAR_REGULAR);
    auto waveField = system.GetEnvironment()->GetFreeSurface()->GetLinearWaveField();
    waveField->SetRegularWaveHeight(0.05);
    waveField->SetRegularWavePeriod(1.397);
    waveField->SetMeanWaveDirection(180.);
    waveField->GetSteadyElevation(0, 0);

    // ------------------------------------------------------
    // Body
    // ------------------------------------------------------

    auto ship = DTMB5512(&system);

    auto vspeed = ChVector<>(1.532, 0., 0.);
    ship->SetPos_dt(vspeed);
    ship->SetSteadyVelocity(vspeed);
    //ship->SetRot(euler_to_quat(chrono::ChVector<double>(0., 0., CH_C_PI)));
    ship->SetDOF(false, true, true, true, true, true);

    // ------------------------------------------------------
    // Monitoring waveProbe
    // ------------------------------------------------------

    auto dynamic_waveProbe = std::make_shared<FrLinearWaveProbe>();
    dynamic_waveProbe->AttachedNode(ship->GetEquilibriumFrame());
    waveField->AddWaveProbe(dynamic_waveProbe);
    dynamic_waveProbe->Initialize();

    auto fixed_waveProbe = waveField->NewWaveProbe(0, 0);
    fixed_waveProbe->Initialize();


    // ------------------------------------------------------
    // Simulation
    // ------------------------------------------------------

    std::vector<ChVector<double>> vposition;
    std::vector<ChVector<double>> vrotation;
    std::vector<ChVector<double>> vforce;
    std::vector<ChVector<double>> vtorque;
    std::vector<double> waveElevation0, waveElevation1;
    std::vector<double> vtime;

    double dt = 0.01;

    system.SetTimestepperType(chrono::ChTimestepper::Type::EULER_IMPLICIT);
    system.SetStep(dt);
    system.Initialize();

    auto is_irrlicht = false;

    if (is_irrlicht) {

        auto app = FrIrrApp(system);
        app.AddTypicalCamera(irr::core::vector3df(0, 0, 10), irr::core::vector3df(0, 0, -1));

        app.Run();

    } else {

        ChVector<double> global_force, local_force;
        ChVector<double> global_torque, local_torque;

        auto time = 0.;

        while (time < 50.) {

            // Do step

            system.DoStepDynamics(dt);
            time += dt;
            ship->Update();
            vtime.push_back(time);

            // Save ship state

            vposition.push_back(ship->GetPosition());
            vrotation.push_back(quat_to_euler(ship->GetRot()));

            global_force = ChVector<double>();
            for (auto& force: ship->GetForceList()) {
                force->GetBodyForceTorque(local_force, local_torque);
                global_force += local_force;
                global_torque += local_torque;
            }
            vforce.push_back(global_force);
            vtorque.push_back(global_torque);

            // Monitor wave elevation

            waveElevation0.push_back(fixed_waveProbe->GetElevation(time));
            waveElevation1.push_back(dynamic_waveProbe->GetElevation(time));

        }

        // Adimentionalize

        auto adim_x = 1./(0.5*1025*1.532*1.532*1.4);

        for (auto& force: vforce) {
            force.x() = adim_x * force.x();
            force.z() = adim_x * force.z();
        }

        PlotResult(vtime, vposition, vrotation, 0);
        PlotResult(vtime, vforce, vtorque, 1);

        matplotlibcpp::named_plot("fixed", vtime, waveElevation0);
        matplotlibcpp::named_plot("dynamic", vtime, waveElevation1);
        matplotlibcpp::xlabel("time (s)");
        matplotlibcpp::ylabel("wave elevation (m)");
        matplotlibcpp::legend();
        matplotlibcpp::show();

    }

// ==========================================================
}

